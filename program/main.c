/* 
** By Valentín Pérez Cerutti
**
** I2S driver based on the work of Ricardo Massaro
** https://github.com/moefh/
**/

/*
 * Prueba n3:
 * - Usar timers como osciladores
 *
 */

#include "pico/stdlib.h"
#include <synth_sequencer.h>
#include "hardware/pwm.h"
#include "hardware/interp.h"

#include <math.h>


  /*   MIDI over UART   */

#define UART_ID uart0
#define BAUD_RATE 31250
#define LED_PIN 25
#define MIDI_CC_CANTIDAD 8

volatile uint8_t MIDI_byte = 0, MIDI_msgFlag = 0;
volatile uint8_t MIDI_byte0 = 0x90, MIDI_byte1 = 0, MIDI_byte2 = 0;

volatile uint8_t MIDI_cc[MIDI_CC_CANTIDAD];

void uart_irq_handler() {
  uint8_t byte = uart_getc(UART_ID);  // Leer el byte recibido
  uint8_t byte_channel = byte & 0xf0; // byte sin el canal MIDI

  if(byte_channel == 0xf0) return;    // si es un mensaje de sistema lo omito (start, stop, beat)
  if(byte > 0x7f) MIDI_byte = 0;      // si es un note on o note off reinicio la máquina de estados
  

  switch(MIDI_byte){
      case 0:
          MIDI_byte0 = byte_channel;
          MIDI_byte = 1;
          return;
      
      case 1:
          MIDI_byte1 = byte;
          MIDI_byte = 2;
          return;

      case 2:
          MIDI_byte2 = byte;
          MIDI_byte = 1; // running-mode

          MIDI_msgFlag = 1;
          return;
  }

  // si llegó hasta acá es porque hubo un error
  MIDI_byte = 1;


}

void setup_uart() {
  uart_init(UART_ID, BAUD_RATE);  // Configurar el UART con la velocidad de 31,250 baudios
  gpio_set_function(1, GPIO_FUNC_UART);  // Configurar el pin 1 como RX

  // Habilitar interrupciones en el UART para recibir datos
  uart_set_irq_enables(UART_ID, true, false);  // Habilitar interrupciones cuando hay datos disponibles

  // Configurar la interrupción
  irq_set_exclusive_handler(UART0_IRQ, uart_irq_handler);  // Asociar la interrupción con el manejador
  irq_set_enabled(UART0_IRQ, true);  // Habilitar la interrupción en el sistema
}




  /*   Configuración del I2S   */

#define SAMPLE_RATE 44100

// I2S_SAMPLE_NUM = 1
#define I2S_DATA_PIN             28 // -> I2S DIN
#define I2S_CLOCK_PIN_BASE       26 // -> I2S BCK
// The third required connection is GPIO 27 -> I2S LRCK (BCK+1)

static const struct sound_i2s_config sound_config = {
  .pin_sda         = I2S_DATA_PIN,
  .pin_scl         = I2S_CLOCK_PIN_BASE,
  .pin_ws          = I2S_CLOCK_PIN_BASE + 1,
  .sample_rate     = SAMPLE_RATE,
  .bits_per_sample = 16,
  .pio_num         = 0, // 0 for pio0, 1 for pio1
};



  /*   Interpolador   */
  
void interp_init(){
  interp_config cfg = interp_default_config();
  interp_config_set_blend(&cfg, true);
  interp_set_config(interp0, 0, &cfg);

  cfg = interp_default_config();
  interp_set_config(interp0, 1, &cfg);
}

// alpha es uint8_t {0 - 255}
#define interpolate(b0, b1, alpha, result){ \
  interp0->base[0] = b0;      \
  interp0->base[1] = b1;      \
  interp0->accum[1] = alpha;  \
  *result = (int32_t) interp0->peek;    \
}

void interpolate2(int16_t b0, int16_t  b1, uint8_t alpha, int32_t* result){
  *result = b0 + (((int32_t)(b1-b0) * alpha) >> 8);
}

const int16_t tabla_seno[] = {0, 6393, 12540, 18205, 23170, 27246, 30274, 32138, 32767, 32138, 30274, 27246, 23170, 18205, 12540, 6393, 0, -6393, -12540, -18205, -23170, -27246, -30274, -32138, -32767, -32138, -30274, -27246, -23170, -18205, -12540, -6393, 0};



  /*   LFOs   */

// Para setear la frecuencia hay que decir cada cuántos steps de 44.100 Hz
// queremos que se actualice el valor de salida del LFO (t)
typedef struct{
  volatile uint16_t t, step, step_limit;
  int16_t* tabla;
} LFO;

void LFO_setFreq(LFO* lfo, float freq){
  lfo->step_limit = SAMPLE_RATE / (freq * 32) - 1;  
}

int16_t LFO_get(LFO* lfo){
  lfo->step++;

  if(lfo->step >= lfo->step_limit){
    lfo->step = 0;
    lfo->t = (lfo->t + 1) % 32;
  }

  return lfo->tabla[lfo->t];
}

LFO vibrato = {0, 0, 0, tabla_seno};


  /*   Manejo de voces   */

#define VOCES_CANTIDAD 12
#define DRAW_BARS 8

typedef enum {OFF, ON} voice_state;

typedef struct{
  volatile voice_state state;
  volatile uint8_t note, oct;
  volatile uint16_t wrap;
  volatile float freq;
} voice_struct;

voice_struct voice[VOCES_CANTIDAD];

volatile uint8_t draw_bar[DRAW_BARS];

// Estructura del envolvente: ADSR
typedef struct {
  uint16_t att, dec, sus, rel;
} ADSR;
// att, dec y rel se expresan en 'ticks'. sus en amplitud

// ADSRs
volatile ADSR VCA_env = {0x0100, 0x0001, 0xffff, 0x0200};


voice_struct* voices_priority[VOCES_CANTIDAD];
// voice_priority es una lista de punteros (a voces) que dicen qué voz está hace más
// tiempo inactiva, para que el algoritmo de selección de voces la elija. Luego de esto,
// esta voz pasa al final de todo. Si una voz termina su 'release' pasa al principio de
// la lista para ser elegida nuevamente.

void voices_priority_reset(){
  for(uint8_t i = VOCES_CANTIDAD; i < VOCES_CANTIDAD; i++){
    voice[i].state = OFF;
  }

  for(uint8_t i = 0; i < VOCES_CANTIDAD; i++){
    voices_priority[i] = &voice[i];
  }
}

// se encarga de decirle al algoritmo qué voz debe elegir
voice_struct* next_voice(){
  voice_struct* _voz = voices_priority[0];

  for(uint8_t i = 0; i < VOCES_CANTIDAD - 1; i++){
    voices_priority[i] = voices_priority[i+1];
  }
  voices_priority[VOCES_CANTIDAD - 1] = _voz;

  return _voz;
}

// Funciones para iniciar o parar una voz
void trigger_attack(voice_struct *voice_pointer){
  uint8_t voice_num = (uint8_t)(voice_pointer - voice);
  pwm_set_counter(voice_num, 0);
  

  voice_pointer->state = ON;
}

void trigger_release(voice_struct *voice_pointer){
  voice_pointer->state = OFF;

  bool flag_swapped = false;
  for(uint8_t i = VOCES_CANTIDAD - 1; i > 0; i--){
    if(voices_priority[i] == voice_pointer)
      flag_swapped = true;

    if(flag_swapped)
      voices_priority[i] = voices_priority[i - 1];
    
  }

  voices_priority[0] = voice_pointer;
}



// Función para actualizar las envolventes
void voices_VCA_env_actualizar(){
  for(uint8_t i = 0; i < VOCES_CANTIDAD; i++){
    
  }
}


  /*   Modificadores   */

  uint8_t detune = 0;


  /*   Timers de PWM   */

  #define TIMERS_CANTIDAD 12

  void pwm_setup(){
  
    pwm_config config = pwm_get_default_config();
  
  
    // habilito los 12 PWMs y establezco el wrap en 0xffff
    for(uint8_t k = 0; k < TIMERS_CANTIDAD; k++){
      pwm_set_wrap(k, 0x7fff);
      pwm_set_enabled(k, true);
      pwm_set_clkdiv(k, 1);
    }
  
    // podría subir la octava de oscilador cambiando el wrap a 2^15 ó 2^14
    // para modificar la frecuencia cambio el prescaler con:
    // pwm_set_clkdiv(slice, div), siendo div un float >= 1.f y < 256.f
    // y obtengo el valor actual del timer con:
    // pwm_get_counter(slice);
  
  }


//const uint16_t MIDI_to_freq[] = {65, 69, 73, 78, 82, 87, 92, 98, 104, 110, 117, 123, 131, 139, 147, 156, 165, 175, 185, 196, 208, 220, 233, 247, 262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494, 523, 554, 587, 622, 659, 698, 740, 784, 831, 880, 932, 988, 1046};
// empieza desde C2 = 36

const float MIDI_to_freq[] = {32.703, 34.648, 36.708, 38.891, 41.203, 43.653, 46.249, 48.999, 51.913, 55.000, 58.270, 61.735, 65.406, 69.295, 73.416, 77.781, 82.406, 87.307, 92.498, 97.998, 103.826, 109.999, 116.540, 123.470, 130.812, 138.590, 146.832, 155.563, 164.813, 174.613, 184.996, 195.997, 207.651, 219.999, 233.080, 246.940, 261.624, 277.181, 293.663, 311.125, 329.626, 349.226, 369.992, 391.993, 415.302, 439.997, 466.161, 493.880, 523.248, 554.362, 587.326, 622.250, 659.251, 698.452, 739.984, 783.986, 830.604, 879.995, 932.322, 987.761, 1046.496, 1108.724, 1174.652, 1244.500, 1318.502, 1396.905, 1479.969, 1567.972, 1661.209, 1759.989, 1864.644, 1975.521, 2092.992, 2217.448, 2349.304, 2489.001, 2637.005, 2793.809, 2959.938, 3135.945, 3322.418, 3519.979, 3729.288, 3951.043, 4185.984};
// empieza desde C1 = 24

void timers_refresh(){
  for(uint8_t i = 0; i < VOCES_CANTIDAD; i++){
    if(voice[i].state != OFF){
      // Calculo en qué octava estoy 
      voice[i].oct = (voice[i].note - 24)/12;
      // Calculo la frecuencia
      voice[i].freq = (SYS_CLK_HZ >> 15) / (float)(MIDI_to_freq[(voice[i].note - 24)%12]);

      // Asigno el prescaler a todos los timers que corresponda
      // pwm_set_clkdiv(i, voice[i].freq); <- lo hago en tiempo real para el vibrato
      // y el wrap
      voice[i].wrap = (0x7fff) >> voice[i].oct;
      pwm_set_wrap(i, voice[i].wrap);

    }
  }
}

void saw2sine(uint16_t timer_count_, uint16_t timer_wrap, int32_t* result){
  uint16_t timer_count = timer_count_ % timer_wrap;
  uint8_t base = timer_count / (timer_wrap >> 5);
  uint8_t alpha = ((timer_count % (timer_wrap >> 5)) << 8) / (timer_wrap >> 5);
  interpolate2(tabla_seno[base], tabla_seno[base + 1], alpha, result);
}

// devuelve el valor actual (suma de osciladores) de una voz
int32_t calculate_voice(uint8_t k){
  int32_t draw[DRAW_BARS], sample = 0;
  
  saw2sine(pwm_get_counter(k), voice[k].wrap, &draw[0]);      // Do 0
  saw2sine(pwm_get_counter(k), voice[k].wrap / 3, &draw[1]);  // Sol 1
  saw2sine(pwm_get_counter(k), voice[k].wrap >> 1, &draw[2]); // Do 1
  saw2sine(pwm_get_counter(k), voice[k].wrap >> 2, &draw[3]); // Do 2
  saw2sine(pwm_get_counter(k), voice[k].wrap / 6, &draw[4]);  // Sol 2
  saw2sine(pwm_get_counter(k), voice[k].wrap >> 3, &draw[5]); // Do 3
  saw2sine(pwm_get_counter(k), voice[k].wrap / 10, &draw[6]); // Mi 3
  saw2sine(pwm_get_counter(k), voice[k].wrap / 12, &draw[7]); // Do 4

  for(uint8_t i = 0; i < DRAW_BARS; i++){
    sample += (draw[i] * draw_bar[i]) >> 8;
  }

  return sample;
}



/*   Salida de audio   */

#define int16_max (int16_t) 32767
#define int16_min (int16_t) -32768

#define TREMOLO_DEPTH 9
const float VIBRATO_DEPTH = .01f/128;

int16_t nuevo_sample(){
  int32_t sample = 0;
  uint8_t voces_activas = 0;

  int16_t lfo_vibrato = LFO_get(&vibrato);

  for(uint8_t k = 0; k < VOCES_CANTIDAD; k++){
    if(voice[k].state != OFF){
        
      sample += calculate_voice(k);
      voces_activas++;

      // modificar frecuencia por vibrato
      pwm_set_clkdiv(k, voice[k].freq * (1 + VIBRATO_DEPTH*(lfo_vibrato >> 8)));

    }
  }

  // normalización
  sample = (sample >> 1) / voces_activas;
  
  // tremolo
  sample = (sample * ((int16_t)(1 << TREMOLO_DEPTH) + (lfo_vibrato >> 8))) >> TREMOLO_DEPTH;

  // bajada a 16 bits
  //sample = sample >> 3;
  // saturación
  uint16_t sample_16 = sample > int16_max? int16_max : sample < int16_min? int16_min : sample;

  // Salida sin filtrar
  return sample_16;
}
  


  /*   Timer de audio   */

repeating_timer_t audio_timer;

bool audio_timer_callback(repeating_timer_t *timer) {
  
  static int16_t *last_buffer;
  int16_t *buffer = sound_i2s_get_next_buffer();
  if (buffer == NULL || buffer == last_buffer) { return true; }
    last_buffer = buffer;
  
  // Actualizo las envolventes
  //voices_VCA_env_actualizar();
  // y calculo el nuevo sample
  int16_t level = nuevo_sample();
  
  // Copy to I2S buffer
  *buffer++ = level;
  *buffer++ = level;
      
  return true;
}


/*******************************************************************************/

int main() {

  stdio_init_all();

  setup_uart();
  pwm_setup();
  interp_init();
  sound_i2s_init(&sound_config);
  voices_priority_reset();
  
  LFO_setFreq(&vibrato, 6.7f);

  // Inicializo el audio
  sound_i2s_playback_start();
  add_repeating_timer_us(10, audio_timer_callback, NULL, &audio_timer);



  while (true) {
    
    if(MIDI_msgFlag){
      if(MIDI_byte0 == 0x90){ // Note ON
        voice_struct *voice_pointer = next_voice();

        voice_pointer->note = MIDI_byte1;
        trigger_attack(voice_pointer);

        timers_refresh();
      }

      if(MIDI_byte0 == 0x80){ // Note OFF
        for(volatile uint8_t k = 0; k < VOCES_CANTIDAD; k++){
          if(voice[k].note == MIDI_byte1){
            trigger_release(&voice[k]);
          }
        }

        timers_refresh();
      }

      if(MIDI_byte0 == 0xB0){ // Control Change (CC)
        if(MIDI_byte1 < DRAW_BARS)
          draw_bar[MIDI_byte1] = MIDI_byte2 << 1;

      }
    

      MIDI_msgFlag = 0;
    }
    
  }

  

  return 0;
}



