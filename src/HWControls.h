// This optional setting causes Encoder to use more optimized code,
// It must be defined before Encoder.h is included.
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <Bounce.h>
#include "TButton.h"
#include <ADC.h>
#include <ADC_util.h>

ADC *adc = new ADC();

//Teensy 4.1 - Mux Pins
#define MUX_0 30
#define MUX_1 31
#define MUX_2 32

#define MUX1_S A4  // ADC1
#define MUX2_S A3  // ADC1
#define MUX3_S A2  // ADC1
#define MUX4_S A1  // ADC1
#define MUX5_S A0  // ADC1

//Mux 1 Connections
#define MUX1_osc1_PW 0
#define MUX1_osc2_freq 1
#define MUX1_osc2_eg_depth 2
#define MUX1_osc1_PWM 3
#define MUX1_osc1_level 4
#define MUX1_osc2_detune 5
#define MUX1_osc2_level 6
#define MUX1_spare7 7

//Mux 2 Connections

#define MUX2_vcf_cutoff 0
#define MUX2_vcf_res 1
#define MUX2_lfo1_depth 2
#define MUX2_lfo1_speed 3
#define MUX2_vcf_eg_depth 4
#define MUX2_vcf_key_follow 5
#define MUX2_lfo2_depth 6
#define MUX2_lfo2_speed 7

//Mux 3 Connections

#define MUX3_lfo1_delay 0
#define MUX3_lfo2_delay 1
#define MUX3_lfo3_delay 2
#define MUX3_spare3 3
#define MUX3_vcf_key_velocity 4
#define MUX3_vcf_hpf 5
#define MUX3_lfo3_depth 6
#define MUX3_lfo3_speed 7

//Mux 4 Connections

#define MUX4_eg1_attack 0
#define MUX4_eg1_decay 1
#define MUX4_eg1_sustain 2
#define MUX4_eg1_release 3
#define MUX4_eg2_attack 4
#define MUX4_eg2_decay 5
#define MUX4_eg2_sustain 6
#define MUX4_eg2_release 7

//Mux 5 Connections

#define MUX5_eg1_key_follow 0
#define MUX5_vca_key_velocity 1
#define MUX5_vca_level 2
#define MUX5_eg2_key_follow 3
#define MUX5_spare4 4
#define MUX5_spare5 5
#define MUX5_spare6 4
#define MUX5_spare7 5

// 74HC165 buttons

#define OSC1_OCTAVE 0
#define OSC1_WAVE 1
#define OSC1_SUB 2
#define OSC2_WAVE 3
#define OSC2_XMOD 4
#define OSC2_EG_SELECT 5
#define spare6 6
#define spare7 7

#define LFO1_WAVE 8
#define LFO2_WAVE 9
#define LFO3_WAVE 10
#define LFO_SELECT 11
#define EG_SELECT 12
#define spare13 13
#define spare14 14
#define spare15 15

// 75HC595 pins

#define OSC1_OCTAVE_LED_RED 0
#define OSC1_OCTAVE_LED_GREEN 1
#define OSC1_WAVE_LED_RED 2
#define OSC1_WAVE_LED_GREEN 3
#define OSC1_SUB_LED 4
#define OSC2_WAVE_LED_RED 5
#define OSC2_WAVE_LED_GREEN 6
#define OSC2_XMOD_LED_RED 7

#define OSC2_XMOD_LED_GREEN 8
#define OSC2_EG_SELECT_LED_RED 9
#define OSC2_EG_SELECT_LED_GREEN 10
#define LFO1_WAVE_LED_RED 11
#define LFO1_WAVE_LED_GREEN 12
#define LFO2_WAVE_LED_RED 13
#define LFO2_WAVE_LED_GREEN 14
#define spare15_LED 15

#define LFO3_WAVE_LED_RED 16
#define LFO3_WAVE_LED_GREEN 17
#define LFO_SELECT_LED_RED 18
#define LFO_SELECT_LED_GREEN 19
#define EG_DEST_LED_RED 20
#define EG_DEST_LED_GREEN 21
#define spare22_LED 22
#define spare23_LED 23

//Teensy 4.1 Pins

#define RECALL_SW 40
#define SAVE_SW 41
#define SETTINGS_SW 39
#define BACK_SW 38

#define ENCODER_PINA 5
#define ENCODER_PINB 4

#define MUXCHANNELS 8
#define QUANTISE_FACTOR 31

#define DEBOUNCE 30

static byte muxInput = 0;
static int mux1ValuesPrev[MUXCHANNELS] = {};
static int mux2ValuesPrev[MUXCHANNELS] = {};
static int mux3ValuesPrev[MUXCHANNELS] = {};
static int mux4ValuesPrev[MUXCHANNELS] = {};
static int mux5ValuesPrev[MUXCHANNELS] = {};

static int mux1Read = 0;
static int mux2Read = 0;
static int mux3Read = 0;
static int mux4Read = 0;
static int mux5Read = 0;

static long encPrevious = 0;

//These are pushbuttons and require debouncing

TButton saveButton{ SAVE_SW, LOW, HOLD_DURATION, DEBOUNCE, CLICK_DURATION };
TButton settingsButton{ SETTINGS_SW, LOW, HOLD_DURATION, DEBOUNCE, CLICK_DURATION };
TButton backButton{ BACK_SW, LOW, HOLD_DURATION, DEBOUNCE, CLICK_DURATION };
TButton recallButton{ RECALL_SW, LOW, HOLD_DURATION, DEBOUNCE, CLICK_DURATION }; // on encoder

Encoder encoder(ENCODER_PINB, ENCODER_PINA);  //This often needs the pins swapping depending on the encoder

void setupHardware() {
  //Volume Pot is on ADC0
  adc->adc0->setAveraging(32);                                          // set number of averages 0, 4, 8, 16 or 32.
  adc->adc0->setResolution(12);                                         // set bits of resolution  8, 10, 12 or 16 bits.
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_LOW_SPEED);  // change the conversion speed
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);           // change the sampling speed

  //MUXs on ADC1
  adc->adc1->setAveraging(32);                                          // set number of averages 0, 4, 8, 16 or 32.
  adc->adc1->setResolution(12);                                         // set bits of resolution  8, 10, 12 or 16 bits.
  adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_LOW_SPEED);  // change the conversion speed
  adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);           // change the sampling speed

  //Mux address pins

  pinMode(MUX_0, OUTPUT);
  pinMode(MUX_1, OUTPUT);
  pinMode(MUX_2, OUTPUT);

  digitalWrite(MUX_0, LOW);
  digitalWrite(MUX_1, LOW);
  digitalWrite(MUX_2, LOW);


  //Mux ADC
  pinMode(MUX1_S, INPUT_DISABLE);
  pinMode(MUX2_S, INPUT_DISABLE);
  pinMode(MUX3_S, INPUT_DISABLE);
  pinMode(MUX4_S, INPUT_DISABLE);
  pinMode(MUX5_S, INPUT_DISABLE);

  //Switches
  pinMode(RECALL_SW, INPUT_PULLUP);  //On encoder
  pinMode(SAVE_SW, INPUT_PULLUP);
  pinMode(SETTINGS_SW, INPUT_PULLUP);
  pinMode(BACK_SW, INPUT_PULLUP);

}
