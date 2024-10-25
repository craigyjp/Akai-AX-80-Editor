/*
  AX80 Editor - Firmware Rev 1.0

  Includes code by:
    Dave Benn - Handling MUXs, a few other bits and original inspiration  https://www.notesandvolts.com/2019/01/teensy-synth-part-10-hardware.html
    ElectroTechnique for general method of menus and updates.

  Arduino IDE
  Tools Settings:
  Board: "Teensy4,1"
  USB Type: "Serial + MIDI"
  CPU Speed: "600"
  Optimize: "Fastest"

  Performance Tests   CPU  Mem
  180Mhz Faster       81.6 44
  180Mhz Fastest      77.8 44
  180Mhz Fastest+PC   79.0 44
  180Mhz Fastest+LTO  76.7 44
  240MHz Fastest+LTO  55.9 44

  Additional libraries:
    Agileware CircularBuffer available in Arduino libraries manager
    Replacement files are in the Modified Libraries folder and need to be placed in the teensy Audio folder.
*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <MIDI.h>
#include "MidiCC.h"
#include "Constants.h"
#include "Parameters.h"
#include "PatchMgr.h"
#include "HWControls.h"
#include "EepromMgr.h"
#include <RoxMux.h>

#define PARAMETER 0      //The main page for displaying the current patch and control (parameter) changes
#define RECALL 1         //Patches list
#define SAVE 2           //Save patch page
#define REINITIALISE 3   // Reinitialise message
#define PATCH 4          // Show current patch bypassing PARAMETER
#define PATCHNAMING 5    // Patch naming page
#define DELETE 6         //Delete patch page
#define DELETEMSG 7      //Delete patch message page
#define SETTINGS 8       //Settings page
#define SETTINGSVALUE 9  //Settings page

unsigned int state = PARAMETER;

#include "ST7735Display.h"

boolean cardStatus = false;

//MIDI 5 Pin DIN
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);

byte ccType = 2;  //(EEPROM)

#include "Settings.h"

#define OCTO_TOTAL 2
#define BTN_DEBOUNCE 50
RoxOctoswitch<OCTO_TOTAL, BTN_DEBOUNCE> octoswitch;

// pins for 74HC165
#define PIN_DATA 25  // pin 9 on 74HC165 (DATA)
#define PIN_LOAD 28  // pin 1 on 74HC165 (LOAD)
#define PIN_CLK 29   // pin 2 on 74HC165 (CLK))

#define SRP_TOTAL 3
Rox74HC595<SRP_TOTAL> srp;

// pins for 74HC595
#define LED_DATA 35   // pin 14 on 74HC595 (DATA)
#define LED_CLK 33    // pin 11 on 74HC595 (CLK)
#define LED_LATCH 34  // pin 12 on 74HC595 (LATCH)
#define LED_PWM -1    // pin 13 on 74HC595

int count = 0;  //For MIDI Clk Sync
int DelayForSH3 = 12;
int patchNo = 1;               //Current patch no
int voiceToReturn = -1;        //Initialise
long earliestTime = millis();  //For voice allocation - initialise to now

byte byteArray[8];
byte writeRequest[7];
byte saveRequest[6];
byte sysexBuff(32);
byte data(32);

void setup() {
  SPI.begin();
  setupDisplay();

  octoswitch.begin(PIN_DATA, PIN_LOAD, PIN_CLK);
  octoswitch.setCallback(onButtonPress);

  srp.begin(LED_DATA, LED_LATCH, LED_CLK, LED_PWM);

  setUpSettings();
  setupHardware();

  byteArray[0] = 0xF0;  // Start of SysEx
  byteArray[1] = 0x42;  // Manufacturer ID (example value)
  byteArray[2] = 0x30;  // Data byte 1 (example value)
  byteArray[3] = 0x04;  // Data byte 2 (example value)
  byteArray[4] = 0x41;  // Parameter Change
  byteArray[5] = 0x00;
  byteArray[6] = 0x00;
  byteArray[7] = 0xF7;  // End of Exclusive

  writeRequest[0] = 0xF0;  // Start of SysEx
  writeRequest[1] = 0x42;  // Manufacturer ID (example value)
  writeRequest[2] = 0x30;  // Format ID 42H
  writeRequest[3] = 0x04;  // DW6000 ID
  writeRequest[4] = 0x11;  // Write request
  writeRequest[5] = 0x00;
  writeRequest[6] = 0xF7;  // End of Exclusive

  saveRequest[0] = 0xF0;  // Start of SysEx
  saveRequest[1] = 0x42;  // Manufacturer ID (example value)
  saveRequest[2] = 0x30;  // Format ID 42H
  saveRequest[3] = 0x04;  // DW6000 ID
  saveRequest[4] = 0x10;  // Write request
  saveRequest[5] = 0xF7;  // End of Exclusive

  cardStatus = SD.begin(BUILTIN_SDCARD);
  if (cardStatus) {
    Serial.println("SD card is connected");
    //Get patch numbers and names from SD card
    loadPatches();
    if (patches.size() == 0) {
      //save an initialised patch to SD card
      savePatch("1", INITPATCH);
      loadPatches();
    }
  } else {
    Serial.println("SD card is not connected or unusable");
    reinitialiseToPanel();
    showPatchPage("No SD", "conn'd / usable");
  }

  //Read MIDI Channel from EEPROM
  midiChannel = getMIDIChannel();
  Serial.println("MIDI Ch:" + String(midiChannel) + " (0 is Omni On)");

  //Read UpdateParams type from EEPROM
  updateParams = getUpdateParams();

  //USB Client MIDI
  usbMIDI.setHandleControlChange(myConvertControlChange);
  usbMIDI.setHandleProgramChange(myProgramChange);
  usbMIDI.setHandleNoteOff(myNoteOff);
  usbMIDI.setHandleNoteOn(myNoteOn);
  usbMIDI.setHandleSystemExclusive(mySystemExclusiveChunk);
  Serial.println("USB Client MIDI Listening");

  //MIDI 5 Pin DIN
  MIDI.begin();
  MIDI.setHandleControlChange(myConvertControlChange);
  MIDI.setHandleProgramChange(myProgramChange);
  MIDI.setHandleNoteOn(myNoteOn);
  MIDI.setHandleNoteOff(myNoteOff);
  MIDI.setHandleSystemExclusive(mySystemExclusiveChunk);
  Serial.println("MIDI In DIN Listening");

  //Read Encoder Direction from EEPROM
  encCW = getEncoderDir();

  //Read MIDI Out Channel from EEPROM
  midiOutCh = getMIDIOutCh();

  recallPatch(patchNo);  //Load first patch
}

void myNoteOn(byte channel, byte note, byte velocity) {
  MIDI.sendNoteOn(note, velocity, channel);
}

void myNoteOff(byte channel, byte note, byte velocity) {
  MIDI.sendNoteOff(note, velocity, channel);
}

void checkLoadFromDW() {
  if (loadFromDW) {
    if (!dataInProgress) {
      if (midiOutCh > 0) {
        MIDI.sendProgramChange(currentSendPatch, midiOutCh);
        delay(100);
        MIDI.sendSysEx(sizeof(saveRequest), saveRequest);
        dataInProgress = true;
      }
    }
  }
}

void mySystemExclusiveChunk(byte *data, unsigned int length) {

  if (loadFromDW) {
    for (unsigned int n = 5; n < 31; n++) {
      recallPatchFlag = true;
      switch (n) {
        // case 5:  // Parameter 0 - Bend Osc - Assign Mode
        //   bend_osc = data[n] & 0x0F;
        //   //updatebend_osc();
        //   polymode = (data[n] >> 4) & 0x03;
        //   switch (polymode) {
        //     case 0:
        //       poly1 = 1;
        //       poly2 = 0;
        //       unison = 0;
        //       //updatePoly1();
        //       break;

        //     case 1:
        //       poly1 = 0;
        //       poly2 = 1;
        //       unison = 0;
        //       //updatePoly2();
        //       break;

        //     case 2:
        //       poly1 = 0;
        //       poly2 = 0;
        //       unison = 1;
        //       //updateUnison();
        //       break;
        //   }
        //   break;

        // case 6:  // Parameter 1 - Portamento Time
        //   glide_time = data[n];
        //   //updateglide_time();
        //   wave_banka = (data[n] >> 5) & 0x03;
        //   break;

        // case 7:  // Parameter 2 - OSC1 Level
        //   osc1_level = data[n];
        //   //updateosc1_level();
        //   wave_bankb = (data[n] >> 6) & 0x01;
        //   break;

        // case 8:  // Parameter 3 - OSC2 Level
        //   osc2_level = data[n];
        //   //updateosc2_level();
        //   break;

        // case 9:  // Parameter 4 - Noise Level
        //   noise = data[n];
        //   //updatenoise();
        //   break;

        // case 10:  // Parameter 5 - Cutoff
        //   vcf_cutoff = data[n];
        //   //updatevcf_cutoff();
        //   break;

        // case 11:  // Parameter 6 - Resonance
        //   vcf_res = data[n];
        //   //updatevcf_res();
        //   break;

        // case 12:  // Parameter 7 - EG Intensity
        //   vcf_eg_intensity = data[n];
        //   //updatevcf_eg_intensity();
        //   break;

        // case 13:  // Parameter 8 - VCF Attack
        //   vcf_attack = data[n];
        //   //updatevcf_attack();
        //   break;

        // case 14:  // Parameter 9 - VCF Decay
        //   vcf_decay = data[n];
        //   //updatevcf_decay();
        //   break;

        // case 15:  // Parameter 10 - VCF Break Point
        //   vcf_breakpoint = data[n];
        //   //updatevcf_breakpoint();
        //   break;

        // case 16:  // Parameter 11 - VCF Slope
        //   vcf_slope = data[n];
        //   //updatevcf_slope();
        //   break;

        // case 17:  // Parameter 12 - VCF Sustain
        //   vcf_sustain = data[n];
        //   //updatevcf_sustain();
        //   break;

        // case 18:  // Parameter 13 - VCF Release
        //   vcf_release = data[n];
        //   //updatevcf_release();
        //   break;

        // case 19:  // Parameter 14 - VCA Attack
        //   vca_attack = data[n];
        //   //updatevca_attack();
        //   break;

        // case 20:  // Parameter 15 - VCA Decay
        //   vca_decay = data[n];
        //   //updatevca_decay();
        //   break;

        // case 21:  // Parameter 16 - VCA Break Point
        //   vca_breakpoint = data[n];
        //   //updatevca_breakpoint();
        //   break;

        // case 22:  // Parameter 17 - VCA Slope
        //   vca_slope = data[n];
        //   //updatevca_slope();
        //   break;

        // case 23:  // Parameter 18 - VCA Sustain - Bend VCF
        //   vca_sustain = data[n] & 0x1F;
        //   //updatevca_sustain();
        //   bend_vcf = (data[n] >> 5) & 0x01;
        //   //updatebend_vcf();
        //   break;

        // case 24:  // Parameter 19 - VCA Release - OSC 1 OCT
        //   vca_release = data[n] & 0x1F;
        //   //updatevca_release();
        //   osc1_octave = (data[n] >> 5) & 0x03;
        //   //updateosc1_octave();
        //   break;

        // case 25:  // Parameter 20 - MG Freq - OSC 2 OCT
        //   mg_frequency = data[n] & 0x1F;
        //   //updatemg_frequency();
        //   osc2_octave = (data[n] >> 5) & 0x03;
        //   //updateosc2_octave();
        //   break;

        // case 26:  // Parameter 21 - MG Delay - KBD TRACK
        //   mg_delay = data[n] & 0x1F;
        //   //updatemg_delay();
        //   vcf_kbdtrack = (data[n] >> 5) & 0x03;
        //   //updatevcf_kbdtrack();
        //   break;

        // case 27:  // Parameter 22 - MG OSC - VCF Polarity
        //   mg_osc = data[n] & 0x1F;
        //   //updatemg_osc();
        //   vcf_polarity = (data[n] >> 5) & 0x01;
        //   //updatevcf_polarity();
        //   break;

        // case 28:  // Parameter 23 - MG VCF - Chorus
        //   mg_vcf = data[n] & 0x1F;
        //   //updatemg_vcf();
        //   chorus = (data[n] >> 5) & 0x01;
        //   //updatechorus();
        //   break;

        // case 29:  // Parameter 24 - OSC 2 Wave - OSC 1 Wave
        //   osc2_waveform = data[n] & 0x0F;
        //   //updateosc2_waveform();
        //   osc1_waveform = (data[n] >> 3) & 0x07;
        //   //updateosc1_waveform();
        //   break;

        // case 30:  // Parameter 25 - OSC 2 Detune - OSC 2 Interval
        //   osc2_detune = data[n] & 0x07;
        //   //updateosc2_detune();
        //   osc2_interval = (data[n] >> 3) & 0x07;
        //   //updateosc2_interval();
        //   break;
      }
    }

    //wave_bank = wave_banka + (wave_bankb << 2);
    //updatewaveBank();
    recallPatchFlag = false;

    patchName = "Patch ";
    patchName += String(currentSendPatch + 1);

    updatePatchname();
    sprintf(buffer, "%d", currentSendPatch + 1);
    savePatch(buffer, getCurrentPatchData());
    currentSendPatch++;
    delay(100);

    if (currentSendPatch == 64) {
      loadPatches();
      loadFromDW = false;
      storeLoadFromDW(loadFromDW);
      settings::decrement_setting_value();
      settings::save_current_value();
      showSettingsPage();
      delay(100);
      state = PARAMETER;
      recallPatch(1);
      MIDI.sendProgramChange(0, midiOutCh);
    }
    dataInProgress = false;

  } else {

    recallPatchFlag = true;

    for (unsigned int n = 5; n < 31; n++) {

      switch (n) {
        // case 5:  // Parameter 0 - Bend Osc - Assign Mode
        //   bend_osc = data[n] & 0x0F;
        //   updatebend_osc();
        //   polymode = (data[n] >> 4) & 0x03;
        //   switch (polymode) {
        //     case 0:
        //       poly1 = 1;
        //       updatePoly1();
        //       break;

        //     case 1:
        //       poly2 = 1;
        //       updatePoly2();
        //       break;

        //     case 2:
        //       unison = 1;
        //       updateUnison();
        //       break;
        //   }
        //   break;

        // case 6:  // Parameter 1 - Portamento Time & Wavebank
        //   glide_time = data[n];
        //   updateglide_time();
        //   wave_banka = (data[n] >> 5) & 0x03;
        //   break;

        // case 7:  // Parameter 2 - OSC1 Level & Wavebank
        //   osc1_level = data[n];
        //   updateosc1_level();
        //   wave_bankb = (data[n] >> 6) & 0x01;
        //   break;

        // case 8:  // Parameter 3 - OSC2 Level
        //   osc2_level = data[n];
        //   updateosc2_level();
        //   break;

        // case 9:  // Parameter 4 - Noise Level
        //   noise = data[n];
        //   updatenoise();
        //   break;

        // case 10:  // Parameter 5 - Cutoff
        //   vcf_cutoff = data[n];
        //   updatevcf_cutoff();
        //   break;

        // case 11:  // Parameter 6 - Resonance
        //   vcf_res = data[n];
        //   updatevcf_res();
        //   break;

        // case 12:  // Parameter 7 - EG Intensity
        //   vcf_eg_intensity = data[n];
        //   updatevcf_eg_intensity();
        //   break;

        // case 13:  // Parameter 8 - VCF Attack
        //   vcf_attack = data[n];
        //   updatevcf_attack();
        //   break;

        // case 14:  // Parameter 9 - VCF Decay
        //   vcf_decay = data[n];
        //   updatevcf_decay();
        //   break;

        // case 15:  // Parameter 10 - VCF Break Point
        //   vcf_breakpoint = data[n];
        //   updatevcf_breakpoint();
        //   break;

        // case 16:  // Parameter 11 - VCF Slope
        //   vcf_slope = data[n];
        //   updatevcf_slope();
        //   break;

        // case 17:  // Parameter 12 - VCF Sustain
        //   vcf_sustain = data[n];
        //   updatevcf_sustain();
        //   break;

        // case 18:  // Parameter 13 - VCF Release
        //   vcf_release = data[n];
        //   updatevcf_release();
        //   break;

        // case 19:  // Parameter 14 - VCA Attack
        //   vca_attack = data[n];
        //   updatevca_attack();
        //   break;

        // case 20:  // Parameter 15 - VCA Decay
        //   vca_decay = data[n];
        //   updatevca_decay();
        //   break;

        // case 21:  // Parameter 16 - VCA Break Point
        //   vca_breakpoint = data[n];
        //   updatevca_breakpoint();
        //   break;

        // case 22:  // Parameter 17 - VCA Slope
        //   vca_slope = data[n];
        //   updatevca_slope();
        //   break;

        // case 23:  // Parameter 18 - VCA Sustain - Bend VCF
        //   vca_sustain = data[n] & 0x1F;
        //   updatevca_sustain();
        //   bend_vcf = (data[n] >> 5) & 0x01;
        //   updatebend_vcf();
        //   break;

        // case 24:  // Parameter 19 - VCA Release - OSC 1 OCT
        //   vca_release = data[n] & 0x1F;
        //   updatevca_release();
        //   osc1_octave = (data[n] >> 5) & 0x03;
        //   updateosc1_octave();
        //   break;

        // case 25:  // Parameter 20 - MG Freq - OSC 2 OCT
        //   mg_frequency = data[n] & 0x1F;
        //   updatemg_frequency();
        //   osc2_octave = (data[n] >> 5) & 0x03;
        //   updateosc2_octave();
        //   break;

        // case 26:  // Parameter 21 - MG Delay - KBD TRACK
        //   mg_delay = data[n] & 0x1F;
        //   updatemg_delay();
        //   vcf_kbdtrack = (data[n] >> 5) & 0x03;
        //   updatevcf_kbdtrack();
        //   break;

        // case 27:  // Parameter 22 - MG OSC - VCF Polarity
        //   mg_osc = data[n] & 0x1F;
        //   updatemg_osc();
        //   vcf_polarity = (data[n] >> 5) & 0x01;
        //   updatevcf_polarity();
        //   break;

        // case 28:  // Parameter 23 - MG VCF - Chorus
        //   mg_vcf = data[n] & 0x1F;
        //   updatemg_vcf();
        //   chorus = (data[n] >> 5) & 0x01;
        //   updatechorus();
        //   break;

        // case 29:  // Parameter 24 - OSC 2 Wave - OSC 1 Wave
        //   osc2_waveform = data[n] & 0x0F;
        //   updateosc2_waveform();
        //   osc1_waveform = (data[n] >> 3) & 0x07;
        //   updateosc1_waveform();
        //   break;

        // case 30:  // Parameter 25 - OSC 2 Detune - OSC 2 Interval
        //   osc2_detune = data[n] & 0x07;
        //   updateosc2_detune();
        //   osc2_interval = (data[n] >> 3) & 0x07;
        //   updateosc2_interval();
        //   break;
      }
    }

    //wave_bank = wave_banka + (wave_bankb << 2);
    //updatewaveBank();
    recallPatchFlag = false;

    patchName = "Sysex Patch";
    updatePatchname();
  }
}

void myConvertControlChange(byte channel, byte number, byte value) {
  switch (number) {

    case 0:
      bankselect = value;
      break;

    case 1:
      MIDI.sendControlChange(number, value, channel);
      break;

    case 2:
      MIDI.sendControlChange(number, value, channel);
      break;

    case 7:
      MIDI.sendControlChange(96, value, channel);
      break;

    case 64:
      MIDI.sendControlChange(number, value, channel);
      break;

    default:
      int newvalue = value;
      myControlChange(channel, number, newvalue);
      break;
  }
}

void myPitchBend(byte channel, int bend) {
  MIDI.sendPitchBend(bend, channel);
}

void allNotesOff() {
}

void updateosc1_level() {
  if (!recallPatchFlag) {
    if (osc1_level == 0) {
      showCurrentParameterPage("Osc1 Level", String("Off"));
    } else {
      showCurrentParameterPage("Osc1 Level", String(osc1_level));
    }
  }
  midiCCOut(CCosc1_level, osc1_level);
}

void updateosc1_PW() {
  if (!recallPatchFlag) {
    if (osc1_pw == 0) {
      showCurrentParameterPage("Osc1 PW", String("Off"));
    } else {
      showCurrentParameterPage("Osc1 PW", String(osc1_pw));
    }
  }
  midiCCOut(CCosc1_PW, osc1_pw);
}

void updateosc1_PWM() {
  if (!recallPatchFlag) {
    if (osc1_pwm == 0) {
      showCurrentParameterPage("Osc1 PWM", String("Off"));
    } else {
      showCurrentParameterPage("Osc1 PWM", String(osc1_pwm));
    }
  }
  midiCCOut(CCosc1_PWM, osc1_pwm);
}

void updateosc2_freq() {
  if (!recallPatchFlag) {
    switch (osc2_freq) {
      case 0:
        showCurrentParameterPage("Osc2 Freq", String("16 Foot"));
        break;
      case 12:
        showCurrentParameterPage("Osc2 Freq", String("8 Foot"));
        break;
      case 24:
        showCurrentParameterPage("Osc2 Freq", String("4 Foot"));
        break;
      case 36:
        showCurrentParameterPage("Osc2 Freq", String("2 Foot"));
        break;
    }
  }
  midiCCOut(CCosc2_freq, osc2_freq);
}

void updateosc2_eg_depth() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("Osc2 EG Dep", String(osc2_eg_depth));
  }
  midiCCOut(CCosc2_eg_depth, osc2_eg_depth);
}

void updateosc2_level() {
  if (!recallPatchFlag) {
    if (osc2_level == 0) {
      showCurrentParameterPage("Osc2 Level", String("Off"));
    } else {
      showCurrentParameterPage("Osc2 Level", String(osc2_level));
    }
  }
  midiCCOut(CCosc2_level, osc2_level);
}

void updateosc2_detune() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("Osc2 Detune", String(osc2_detune));
  }
  midiCCOut(CCosc2_detune, osc2_detune);
}

void updatevcf_cutoff() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("VCF Cutoff", String(vcf_cutoff));
  }
  midiCCOut(CCvcf_cutoff, vcf_cutoff);
}

void updatevcf_res() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("VCF Res", String(vcf_res));
  }
  midiCCOut(CCvcf_res, vcf_res);
}

void updatevcf_eg_depth() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("EG Depth", String(vcf_eg_depth));
  }
  midiCCOut(CCvcf_eg_depth, vcf_eg_depth);
}

void updatevcf_key_follow() {
  if (!recallPatchFlag) {
    if (vcf_key_follow == 0) {
      showCurrentParameterPage("Key Follow", String("Off"));
    } else {
      showCurrentParameterPage("Key Follow", String(vcf_key_follow));
    }
  }
  midiCCOut(CCvcf_key_follow, vcf_key_follow);
}

void updatevcf_key_velocity() {
  if (!recallPatchFlag) {
    if (vcf_key_velocity == 0) {
      showCurrentParameterPage("Key Velocity", String("Off"));
    } else {
      showCurrentParameterPage("Key Velocity", String(vcf_key_velocity));
    }
  }
  midiCCOut(CCvcf_key_velocity, vcf_key_velocity);
}

void updatevcf_hpf() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("High Pass", String(vcf_hpf));
  }
  midiCCOut(CCvcf_hpf, vcf_hpf);
}

void updatelfo1_depth() {
  if (!recallPatchFlag) {
    if (lfo1_depth == 0) {
      showCurrentParameterPage("LFO1 Depth", String("Off"));
    } else {
      showCurrentParameterPage("LFO1 Depth", String(lfo1_depth));
    }
  }
  midiCCOut(CClfo_select, 0);
  midiCCOut(CClfo1_depth, lfo1_depth);
}

void updatelfo1_speed() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("LFO1 Speed", String(lfo1_speed));
  }
  midiCCOut(CClfo_select, 0);
  midiCCOut(CClfo1_speed, lfo1_speed);
}

void updatelfo1_delay() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("LFO1 Delay", String(lfo1_delay));
  }
  midiCCOut(CClfo_select, 0);
  midiCCOut(CClfo1_delay, lfo1_delay);
}

void updatelfo2_depth() {
  if (!recallPatchFlag) {
    if (lfo2_depth == 0) {
      showCurrentParameterPage("LFO2 Depth", String("Off"));
    } else {
      showCurrentParameterPage("LFO2 Depth", String(lfo2_depth));
    }
  }
  midiCCOut(CClfo_select, 1);
  midiCCOut(CClfo2_depth, lfo2_depth);
}

void updatelfo2_speed() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("LFO2 Speed", String(lfo2_speed));
  }
  midiCCOut(CClfo_select, 1);
  midiCCOut(CClfo2_speed, lfo2_speed);
}

void updatelfo2_delay() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("LFO2 Delay", String(lfo2_delay));
  }
  midiCCOut(CClfo_select, 1);
  midiCCOut(CClfo2_delay, lfo2_delay);
}

void updatelfo3_depth() {
  if (!recallPatchFlag) {
    if (lfo3_depth == 0) {
      showCurrentParameterPage("LFO3 Depth", String("Off"));
    } else {
      showCurrentParameterPage("LFO3 Depth", String(lfo3_depth));
    }
  }
  midiCCOut(CClfo_select, 2);
  midiCCOut(CClfo3_depth, lfo3_depth);
}

void updatelfo3_speed() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("LFO3 Speed", String(lfo3_speed));
  }
  midiCCOut(CClfo_select, 2);
  midiCCOut(CClfo3_speed, lfo3_speed);
}

void updatelfo3_delay() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("LFO3 Delay", String(lfo3_delay));
  }
  midiCCOut(CClfo_select, 2);
  midiCCOut(CClfo3_delay, lfo3_delay);
}

void updateeg1_attack() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("EG1 Attack", String(eg1_attack));
  }
  midiCCOut(CCeg_select, 0);
  midiCCOut(CCeg1_attack, eg1_attack);
}

void updateeg1_decay() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("EG1 Decay", String(eg1_decay));
  }
  midiCCOut(CCeg_select, 0);
  midiCCOut(CCeg1_decay, eg1_decay);
}

void updateeg1_release() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("EG1 Release", String(eg1_release));
  }
  midiCCOut(CCeg_select, 0);
  midiCCOut(CCeg1_release, eg1_release);
}

void updateeg1_sustain() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("EG1 Sustain", String(eg1_sustain));
  }
  midiCCOut(CCeg_select, 0);
  midiCCOut(CCeg1_sustain, eg1_sustain);
}

void updateeg1_key_follow() {
  if (!recallPatchFlag) {
    if (eg1_key_follow == 0) {
      showCurrentParameterPage("EG1 K.Follow", String("Off"));
    } else {
      showCurrentParameterPage("EG1 K.Follow", String(eg1_key_follow));
    }
  }
  midiCCOut(CCeg_select, 0);
  midiCCOut(CCeg1_key_follow, eg1_key_follow);
}

void updateeg2_attack() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("EG2 Attack", String(eg2_attack));
  }
  midiCCOut(CCeg_select, 2);
  midiCCOut(CCeg2_attack, eg2_attack);
}

void updateeg2_decay() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("EG2 Decay", String(eg2_decay));
  }
  midiCCOut(CCeg_select, 2);
  midiCCOut(CCeg2_decay, eg2_decay);
}

void updateeg2_release() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("EG2 Release", String(eg2_release));
  }
  midiCCOut(CCeg_select, 2);
  midiCCOut(CCeg2_release, eg2_release);
}

void updateeg2_sustain() {
  if (!recallPatchFlag) {
    showCurrentParameterPage("EG2 Sustain", String(eg2_sustain));
  }
  midiCCOut(CCeg_select, 2);
  midiCCOut(CCeg2_sustain, eg2_sustain);
}

void updateeg2_key_follow() {
  if (!recallPatchFlag) {
    if (eg2_key_follow == 0) {
      showCurrentParameterPage("EG2 K.Follow", String("Off"));
    } else {
      showCurrentParameterPage("EG2 K.Follow", String(eg2_key_follow));
    }
  }
  midiCCOut(CCeg_select, 2);
  midiCCOut(CCeg2_key_follow, eg2_key_follow);
}

void updatevca_key_velocity() {
  if (!recallPatchFlag) {
    if (vca_key_velocity == 0) {
      showCurrentParameterPage("VCA evel", String("Off"));
    } else {
      showCurrentParameterPage("VCA evel", String(vca_key_velocity));
    }
  }
  midiCCOut(CCvca_key_velocity, vca_key_velocity);
}

void updatevca_level() {
  if (!recallPatchFlag) {
    if (vca_level == 0) {
      showCurrentParameterPage("VCA Level", String("Off"));
    } else {
      showCurrentParameterPage("VCA Level", String(vca_level));
    }
  }
  midiCCOut(CCvca_level, vca_level);
}

// Buttons

void updateosc1_octave() {
  if (!recallPatchFlag) {
    switch (osc1_octave) {
      case 2:
        showCurrentParameterPage("Osc1 Octave", String("4 Foot"));
        break;
      case 1:
        showCurrentParameterPage("Osc1 Octave", String("8 Foot"));
        break;
      case 0:
        showCurrentParameterPage("Osc1 Octave", String("16 Foot"));
        break;
    }
  }
  switch (osc1_octave) {
    case 2:
      srp.writePin(OSC1_OCTAVE_LED_RED, HIGH);
      srp.writePin(OSC1_OCTAVE_LED_GREEN, HIGH);
      break;
    case 1:
      srp.writePin(OSC1_OCTAVE_LED_RED, LOW);
      srp.writePin(OSC1_OCTAVE_LED_GREEN, HIGH);
      break;
    case 0:
      srp.writePin(OSC1_OCTAVE_LED_RED, HIGH);
      srp.writePin(OSC1_OCTAVE_LED_GREEN, LOW);
      break;
  }
  midiCCOut(CCosc1_octave, osc1_octave);
}

void updateosc1_wave() {
  if (!recallPatchFlag) {
    switch (osc1_wave) {
      case 0:
        showCurrentParameterPage("Osc1 Wave", String("Off"));
        break;
      case 1:
        showCurrentParameterPage("Osc1 Wave", String("Sawtooth"));
        break;
      case 2:
        showCurrentParameterPage("Osc1 Wave", String("Pulse"));
        break;
      case 3:
        showCurrentParameterPage("Osc1 Wave", String("Saw & Pulse"));
        break;
    }
  }
  switch (osc1_wave) {
    case 0:
      srp.writePin(OSC1_WAVE_LED_RED, LOW);
      srp.writePin(OSC1_WAVE_LED_GREEN, LOW);
      break;
    case 1:
      srp.writePin(OSC1_WAVE_LED_RED, HIGH);
      srp.writePin(OSC1_WAVE_LED_GREEN, LOW);
      break;
    case 2:
      srp.writePin(OSC1_WAVE_LED_RED, LOW);
      srp.writePin(OSC1_WAVE_LED_GREEN, HIGH);
      break;
    case 3:
      srp.writePin(OSC1_WAVE_LED_RED, HIGH);
      srp.writePin(OSC1_WAVE_LED_GREEN, HIGH);
      break;
  }
  midiCCOut(CCosc1_wave, osc1_wave);
}

void updateosc1_sub() {
  if (!recallPatchFlag) {
    switch (osc1_sub) {
      case 0:
        showCurrentParameterPage("Osc1 Sub", String("Off"));
        break;
      case 1:
        showCurrentParameterPage("Osc1 Sub", String("On"));
        break;
    }
  }
  switch (osc1_sub) {
    case 0:
      srp.writePin(OSC1_SUB_LED, LOW);
      break;
    case 1:
      srp.writePin(OSC1_SUB_LED, HIGH);
      break;
  }
  midiCCOut(CCosc1_sub, osc1_sub);
}

void updateosc2_wave() {
  if (!recallPatchFlag) {
    switch (osc2_wave) {
      case 0:
        showCurrentParameterPage("Osc2 Wave", String("Off"));
        break;
      case 1:
        showCurrentParameterPage("Osc2 Wave", String("Sawtooth"));
        break;
      case 2:
        showCurrentParameterPage("Osc2 Wave", String("Pulse"));
        break;
      case 3:
        showCurrentParameterPage("Osc2 Wave", String("Saw & Pulse"));
        break;
    }
  }
  switch (osc2_wave) {
    case 0:
      srp.writePin(OSC2_WAVE_LED_RED, LOW);
      srp.writePin(OSC2_WAVE_LED_GREEN, LOW);
      break;
    case 1:
      srp.writePin(OSC2_WAVE_LED_RED, HIGH);
      srp.writePin(OSC2_WAVE_LED_GREEN, LOW);
      break;
    case 2:
      srp.writePin(OSC2_WAVE_LED_RED, LOW);
      srp.writePin(OSC2_WAVE_LED_GREEN, HIGH);
      break;
    case 3:
      srp.writePin(OSC2_WAVE_LED_RED, HIGH);
      srp.writePin(OSC2_WAVE_LED_GREEN, HIGH);
      break;
  }
  midiCCOut(CCosc2_wave, osc2_wave);
}

void updateosc2_xmod() {
  if (!recallPatchFlag) {
    switch (osc2_xmod) {
      case 0:
        showCurrentParameterPage("Cross Mod", String("Off"));
        break;
      case 1:
        showCurrentParameterPage("Cross Mod", String("Osc1 = Osc2"));
        break;
      case 2:
        showCurrentParameterPage("Cross Mod", String("Osc2 >> Osc1"));
        break;
    }
  }
  switch (osc2_xmod) {
    case 0:
      srp.writePin(OSC2_XMOD_LED_RED, LOW);
      srp.writePin(OSC2_XMOD_LED_GREEN, LOW);
      break;
    case 1:
      srp.writePin(OSC2_XMOD_LED_RED, HIGH);
      srp.writePin(OSC2_XMOD_LED_GREEN, LOW);
      break;
    case 2:
      srp.writePin(OSC2_XMOD_LED_RED, LOW);
      srp.writePin(OSC2_XMOD_LED_GREEN, HIGH);
      break;
  }
  midiCCOut(CCosc2_xmod, osc2_xmod);
}

void updateosc2_eg_select() {
  if (!recallPatchFlag) {
    switch (osc2_eg_select) {
      case 0:
        showCurrentParameterPage("EG Select", String("VCF"));
        break;
      case 1:
        showCurrentParameterPage("EG Select", String("VCA"));
        break;
    }
  }
  switch (osc2_eg_select) {
    case 0:
      srp.writePin(OSC2_EG_SELECT_LED_RED, HIGH);
      srp.writePin(OSC2_EG_SELECT_LED_GREEN, LOW);
      break;
    case 1:
      srp.writePin(OSC2_EG_SELECT_LED_RED, LOW);
      srp.writePin(OSC2_EG_SELECT_LED_GREEN, HIGH);
      break;
  }
  midiCCOut(CCosc2_eg_select, osc2_eg_select);
}

void updatelfo1_wave() {
  if (!recallPatchFlag) {
    switch (lfo1_wave) {
      case 0:
        showCurrentParameterPage("LFO1 Wave", String("Pulse"));
        break;
      case 1:
        showCurrentParameterPage("LFO1 Wave", String("Saw Down"));
        break;
      case 2:
        showCurrentParameterPage("LFO1 Wave", String("Saw Up"));
        break;
      case 3:
        showCurrentParameterPage("LFO1 Wave", String("Triangle"));
        break;
    }
  }
  switch (lfo1_wave) {
    case 0:
      srp.writePin(LFO1_WAVE_LED_RED, LOW);
      srp.writePin(LFO1_WAVE_LED_GREEN, LOW);
      break;
    case 1:
      srp.writePin(LFO1_WAVE_LED_RED, HIGH);
      srp.writePin(LFO1_WAVE_LED_GREEN, LOW);
      break;
    case 2:
      srp.writePin(LFO1_WAVE_LED_RED, LOW);
      srp.writePin(LFO1_WAVE_LED_GREEN, HIGH);
      break;
    case 3:
      srp.writePin(LFO1_WAVE_LED_RED, HIGH);
      srp.writePin(LFO1_WAVE_LED_GREEN, HIGH);
      break;
  }
  midiCCOut(CClfo_select, 0);
  midiCCOut(CClfo1_wave, lfo1_wave);
}

void updatelfo2_wave() {
  if (!recallPatchFlag) {
    switch (lfo2_wave) {
      case 0:
        showCurrentParameterPage("LFO2 Wave", String("Pulse"));
        break;
      case 1:
        showCurrentParameterPage("LFO2 Wave", String("Saw Down"));
        break;
      case 2:
        showCurrentParameterPage("LFO2 Wave", String("Saw Up"));
        break;
      case 3:
        showCurrentParameterPage("LFO2 Wave", String("Triangle"));
        break;
    }
  }
  switch (lfo2_wave) {
    case 0:
      srp.writePin(LFO2_WAVE_LED_RED, LOW);
      srp.writePin(LFO2_WAVE_LED_GREEN, LOW);
      break;
    case 1:
      srp.writePin(LFO2_WAVE_LED_RED, HIGH);
      srp.writePin(LFO2_WAVE_LED_GREEN, LOW);
      break;
    case 2:
      srp.writePin(LFO2_WAVE_LED_RED, LOW);
      srp.writePin(LFO2_WAVE_LED_GREEN, HIGH);
      break;
    case 3:
      srp.writePin(LFO2_WAVE_LED_RED, HIGH);
      srp.writePin(LFO2_WAVE_LED_GREEN, HIGH);
      break;
  }
  midiCCOut(CClfo_select, 1);
  midiCCOut(CClfo2_wave, lfo2_wave);
}

void updatelfo3_wave() {
  if (!recallPatchFlag) {
    switch (lfo3_wave) {
      case 0:
        showCurrentParameterPage("LFO3 Wave", String("Pulse"));
        break;
      case 1:
        showCurrentParameterPage("LFO3 Wave", String("Saw Down"));
        break;
      case 2:
        showCurrentParameterPage("LFO3 Wave", String("Saw Up"));
        break;
      case 3:
        showCurrentParameterPage("LFO3 Wave", String("Triangle"));
        break;
    }
  }
  switch (lfo3_wave) {
    case 0:
      srp.writePin(LFO3_WAVE_LED_RED, LOW);
      srp.writePin(LFO3_WAVE_LED_GREEN, LOW);
      break;
    case 1:
      srp.writePin(LFO3_WAVE_LED_RED, HIGH);
      srp.writePin(LFO3_WAVE_LED_GREEN, LOW);
      break;
    case 2:
      srp.writePin(LFO3_WAVE_LED_RED, LOW);
      srp.writePin(LFO3_WAVE_LED_GREEN, HIGH);
      break;
    case 3:
      srp.writePin(LFO3_WAVE_LED_RED, HIGH);
      srp.writePin(LFO3_WAVE_LED_GREEN, HIGH);
      break;
  }
  midiCCOut(CClfo_select, 2);
  midiCCOut(CClfo3_wave, lfo3_wave);
}

void updatelfo_select() {
  if (!recallPatchFlag) {
    switch (lfo_select) {
      case 0:
        showCurrentParameterPage("LFO Display", String("LFO 1"));
        break;
      case 1:
        showCurrentParameterPage("LFO Display", String("LFO 2"));
        break;
      case 2:
        showCurrentParameterPage("LFO Display", String("LFO 3"));
        break;
    }
  }
  switch (lfo_select) {
    case 0:
      srp.writePin(LFO_SELECT_LED_RED, HIGH);
      srp.writePin(LFO_SELECT_LED_GREEN, LOW);
      break;
    case 1:
      srp.writePin(LFO_SELECT_LED_RED, LOW);
      srp.writePin(LFO_SELECT_LED_GREEN, HIGH);
      break;
    case 2:
      srp.writePin(LFO_SELECT_LED_RED, HIGH);
      srp.writePin(LFO_SELECT_LED_GREEN, HIGH);
      break;
  }
  midiCCOut(CClfo_select, lfo_select);
}

void updateeg_select() {
  if (!recallPatchFlag) {
    switch (eg_select) {
      case 0:
        showCurrentParameterPage("EG Display", String("VCA"));
        break;
      case 1:
        showCurrentParameterPage("EG Display", String("VCA - VCF"));
        break;
      case 2:
        showCurrentParameterPage("EG Display", String("VCF"));
        break;
    }
  }
  switch (eg_select) {
    case 0:
      srp.writePin(EG_DEST_LED_RED, HIGH);
      srp.writePin(EG_DEST_LED_GREEN, LOW);
      break;
    case 1:
      srp.writePin(EG_DEST_LED_RED, HIGH);
      srp.writePin(EG_DEST_LED_GREEN, HIGH);
      break;
    case 2:
      srp.writePin(EG_DEST_LED_RED, LOW);
      srp.writePin(EG_DEST_LED_GREEN, HIGH);
      break;
  }
  midiCCOut(CCeg_select, eg_select);
}

void updatePatchname() {
  showPatchPage(String(patchNo), patchName);
}

void myControlChange(byte channel, byte control, int value) {
  switch (control) {

    case CCosc1_PW:
      osc1_pw = value;
      osc1_pw = map(osc1_pw, 0, 127, 0, 99);
      updateosc1_PW();
      break;

    case CCosc1_PWM:
      osc1_pwm = value;
      osc1_pwm = map(osc1_pwm, 0, 127, 0, 99);
      updateosc1_PWM();
      break;

    case CCosc1_level:
      osc1_level = value;
      osc1_level = map(osc1_level, 0, 127, 0, 99);
      updateosc1_level();
      break;

    case CCosc2_freq:
      osc2_freq = value;
      osc2_freq = map(osc2_freq, 0, 127, 0, 47);
      updateosc2_freq();
      break;

    case CCosc2_eg_depth:
      osc2_eg_depth = value;
      osc2_eg_depth = map(osc2_eg_depth, 0, 127, 0, 99);
      updateosc2_eg_depth();
      break;

    case CCosc2_detune:
      osc2_detune = value;
      osc2_detune = map(osc2_detune, 0, 127, 0, 99);
      updateosc2_detune();
      break;

    case CCosc2_level:
      osc2_level = value;
      osc2_level = map(osc2_level, 0, 127, 0, 99);
      updateosc2_level();
      break;

    case CCvcf_cutoff:
      vcf_cutoff = value;
      vcf_cutoff = map(vcf_cutoff, 0, 127, 0, 99);
      updatevcf_cutoff();
      break;

    case CCvcf_res:
      vcf_res = value;
      vcf_res = map(vcf_res, 0, 127, 0, 99);
      updatevcf_res();
      break;

    case CCvcf_eg_depth:
      vcf_eg_depth = value;
      vcf_eg_depth = map(vcf_eg_depth, 0, 127, 0, 99);
      updatevcf_eg_depth();
      break;

    case CCvcf_key_follow:
      vcf_key_follow = value;
      vcf_key_follow = map(vcf_key_follow, 0, 127, 0, 99);
      updatevcf_key_follow();
      break;

    case CCvcf_key_velocity:
      vcf_key_velocity = value;
      vcf_key_velocity = map(vcf_key_velocity, 0, 127, 0, 99);
      updatevcf_key_velocity();
      break;

    case CCvcf_hpf:
      vcf_hpf = value;
      vcf_hpf = map(vcf_hpf, 0, 127, 0, 99);
      updatevcf_hpf();
      break;

    case CClfo1_depth:
      lfo1_depth = value;
      lfo1_depth = map(lfo1_depth, 0, 127, 0, 99);
      updatelfo1_depth();
      break;

    case CClfo1_speed:
      lfo1_speed = value;
      lfo1_speed = map(lfo1_speed, 0, 127, 0, 99);
      updatelfo1_speed();
      break;

    case CClfo1_delay:
      lfo1_delay = value;
      lfo1_delay = map(lfo1_delay, 0, 127, 0, 99);
      updatelfo1_delay();
      break;

    case CClfo2_depth:
      lfo2_depth = value;
      lfo2_depth = map(lfo2_depth, 0, 127, 0, 99);
      updatelfo2_depth();
      break;

    case CClfo2_speed:
      lfo2_speed = value;
      lfo2_speed = map(lfo2_speed, 0, 127, 0, 99);
      updatelfo2_speed();
      break;

    case CClfo2_delay:
      lfo2_delay = value;
      lfo2_delay = map(lfo2_delay, 0, 127, 0, 99);
      updatelfo2_delay();
      break;

    case CClfo3_depth:
      lfo3_depth = value;
      lfo3_depth = map(lfo3_depth, 0, 127, 0, 99);
      updatelfo3_depth();
      break;

    case CClfo3_speed:
      lfo3_speed = value;
      lfo3_speed = map(lfo3_speed, 0, 127, 0, 99);
      updatelfo3_speed();
      break;

    case CClfo3_delay:
      lfo3_delay = value;
      lfo3_delay = map(lfo3_delay, 0, 127, 0, 99);
      updatelfo3_delay();
      break;

    case CCeg1_attack:
      eg1_attack = value;
      eg1_attack = map(eg1_attack, 0, 127, 0, 99);
      updateeg1_attack();
      break;

    case CCeg1_decay:
      eg1_decay = value;
      eg1_decay = map(eg1_decay, 0, 127, 0, 99);
      updateeg1_decay();
      break;

    case CCeg1_release:
      eg1_release = value;
      eg1_release = map(eg1_release, 0, 127, 0, 99);
      updateeg1_release();
      break;

    case CCeg1_sustain:
      eg1_sustain = value;
      eg1_sustain = map(eg1_sustain, 0, 127, 0, 99);
      updateeg1_sustain();
      break;

    case CCeg1_key_follow:
      eg1_key_follow = value;
      eg1_key_follow = map(eg1_key_follow, 0, 127, 0, 99);
      updateeg1_key_follow();
      break;

    case CCeg2_attack:
      eg2_attack = value;
      eg2_attack = map(eg2_attack, 0, 127, 0, 99);
      updateeg2_attack();
      break;

    case CCeg2_decay:
      eg2_decay = value;
      eg2_decay = map(eg2_decay, 0, 127, 0, 99);
      updateeg2_decay();
      break;

    case CCeg2_release:
      eg2_release = value;
      eg2_release = map(eg2_release, 0, 127, 0, 99);
      updateeg2_release();
      break;

    case CCeg2_sustain:
      eg2_sustain = value;
      eg2_sustain = map(eg2_sustain, 0, 127, 0, 99);
      updateeg2_sustain();
      break;

    case CCeg2_key_follow:
      eg2_key_follow = value;
      eg2_key_follow = map(eg2_key_follow, 0, 127, 0, 99);
      updateeg2_key_follow();
      break;

    case CCvca_key_velocity:
      vca_key_velocity = value;
      vca_key_velocity = map(vca_key_velocity, 0, 127, 0, 99);
      updatevca_key_velocity();
      break;

    case CCvca_level:
      vca_level = value;
      vca_level = map(vca_level, 0, 127, 0, 99);
      updatevca_level();
      break;

    case CCosc1_octave:
      osc1_octave = value;
      updateosc1_octave();
      break;

    case CCosc1_wave:
      osc1_wave = value;
      updateosc1_wave();
      break;

    case CCosc1_sub:
      value > 0 ? osc1_sub = 1 : osc1_sub = 0;
      updateosc1_sub();
      break;

    case CCosc2_wave:
      osc2_wave = value;
      updateosc2_wave();
      break;

    case CCosc2_xmod:
      osc2_xmod = value;
      updateosc2_xmod();
      break;

    case CCosc2_eg_select:
      osc2_eg_select = value;
      updateosc2_eg_select();
      break;

    case CClfo1_wave:
      lfo1_wave = value;
      updatelfo1_wave();
      break;

    case CClfo2_wave:
      lfo2_wave = value;
      updatelfo2_wave();
      break;

    case CClfo3_wave:
      lfo3_wave = value;
      updatelfo3_wave();
      break;

    case CClfo_select:
      lfo_select = value;
      updatelfo_select();
      break;

    case CCeg_select:
      eg_select = value;
      updateeg_select();
      break;

    case CCallnotesoff:
      allNotesOff();
      break;
  }
}

void onButtonPress(uint16_t btnIndex, uint8_t btnType) {

  if (btnIndex == OSC1_OCTAVE && btnType == ROX_PRESSED) {
    osc1_octave = osc1_octave + 1;
    if (osc1_octave > 2) {
      osc1_octave = 0;
    }
    myControlChange(midiChannel, CCosc1_octave, osc1_octave);
  }

  if (btnIndex == OSC1_WAVE && btnType == ROX_PRESSED) {
    osc1_wave = osc1_wave + 1;
    if (osc1_wave > 3) {
      osc1_wave = 0;
    }
    myControlChange(midiChannel, CCosc1_wave, osc1_wave);
  }

  if (btnIndex == OSC1_SUB && btnType == ROX_PRESSED) {
    osc1_sub = !osc1_sub;
    myControlChange(midiChannel, CCosc1_sub, osc1_sub);
  }

  if (btnIndex == OSC2_WAVE && btnType == ROX_PRESSED) {
    osc2_wave = osc2_wave + 1;
    if (osc2_wave > 3) {
      osc2_wave = 0;
    }
    myControlChange(midiChannel, CCosc2_wave, osc2_wave);
  }

  if (btnIndex == OSC2_XMOD && btnType == ROX_PRESSED) {
    osc2_xmod = osc2_xmod + 1;
    if (osc2_xmod > 2) {
      osc2_xmod = 0;
    }
    myControlChange(midiChannel, CCosc2_xmod, osc2_xmod);
  }

  if (btnIndex == OSC2_EG_SELECT && btnType == ROX_PRESSED) {
    osc2_eg_select = osc2_eg_select + 1;
    if (osc2_eg_select > 1) {
      osc2_eg_select = 0;
    }
    myControlChange(midiChannel, CCosc2_eg_select, osc2_eg_select);
  }

  if (btnIndex == LFO1_WAVE && btnType == ROX_PRESSED) {
    lfo1_wave = lfo1_wave + 1;
    if (lfo1_wave > 3) {
      lfo1_wave = 0;
    }
    myControlChange(midiChannel, CClfo1_wave, lfo1_wave);
  }

  if (btnIndex == LFO2_WAVE && btnType == ROX_PRESSED) {
    lfo2_wave = lfo2_wave + 1;
    if (lfo2_wave > 3) {
      lfo2_wave = 0;
    }
    myControlChange(midiChannel, CClfo2_wave, lfo2_wave);
  }

  if (btnIndex == LFO3_WAVE && btnType == ROX_PRESSED) {
    lfo3_wave = lfo3_wave + 1;
    if (lfo3_wave > 3) {
      lfo3_wave = 0;
    }
    myControlChange(midiChannel, CClfo3_wave, lfo3_wave);
  }

  if (btnIndex == LFO_SELECT && btnType == ROX_PRESSED) {
    lfo_select = lfo_select + 1;
    if (lfo_select > 2) {
      lfo_select = 0;
    }
    myControlChange(midiChannel, CClfo_select, lfo_select);
  }

  if (btnIndex == EG_SELECT && btnType == ROX_PRESSED) {
    eg_select = eg_select + 1;
    if (eg_select > 2) {
      eg_select = 0;
    }
    myControlChange(midiChannel, CCeg_select, eg_select);
  }
}

void myProgramChange(byte channel, byte program) {
  state = PATCH;
  patchNo = program + 1;
  recallPatch(patchNo);
  Serial.print("MIDI Pgm Change:");
  Serial.println(patchNo);
  state = PARAMETER;
}

void recallPatch(int patchNo) {
  allNotesOff();
  if (!updateParams) {
    MIDI.sendProgramChange(patchNo - 1, midiOutCh);
  }
  delay(50);
  recallPatchFlag = true;
  File patchFile = SD.open(String(patchNo).c_str());
  if (!patchFile) {
    Serial.println("File not found");
  } else {
    String data[NO_OF_PARAMS];  //Array of data read in
    recallPatchData(patchFile, data);
    setCurrentPatchData(data);
    patchFile.close();
  }
  recallPatchFlag = false;
}

void setCurrentPatchData(String data[]) {
  patchName = data[0];
  osc1_octave = data[1].toInt();
  osc1_wave = data[2].toInt();
  osc1_pw = data[3].toInt();
  osc1_pwm = data[4].toInt();
  osc1_sub = data[5].toInt();
  osc1_level = data[6].toInt();
  osc2_freq = data[7].toInt();
  osc2_detune = data[8].toInt();
  osc2_wave = data[9].toInt();
  osc2_xmod = data[10].toInt();
  osc2_eg_depth = data[11].toInt();
  osc2_eg_select = data[12].toInt();
  osc2_level = data[13].toInt();
  vcf_cutoff = data[14].toInt();
  vcf_res = data[15].toInt();
  vcf_eg_depth = data[16].toInt();
  vcf_key_follow = data[17].toInt();
  vcf_key_velocity = data[18].toInt();
  vcf_hpf = data[19].toInt();
  lfo1_depth = data[20].toInt();
  lfo1_speed = data[21].toInt();
  lfo1_delay = data[22].toInt();
  lfo1_wave = data[23].toInt();
  lfo_select = data[24].toInt();
  eg1_attack = data[25].toInt();
  eg1_decay = data[26].toInt();
  eg1_sustain = data[27].toInt();
  eg1_release = data[28].toInt();
  eg1_key_follow = data[29].toInt();
  eg_select = data[30].toInt();
  vca_key_velocity = data[31].toInt();
  vca_level = data[32].toInt();
  lfo2_depth = data[33].toInt();
  lfo2_speed = data[34].toInt();
  lfo2_delay = data[35].toInt();
  lfo2_wave = data[36].toInt();
  lfo3_depth = data[37].toInt();
  lfo3_speed = data[38].toInt();
  lfo3_delay = data[39].toInt();
  lfo3_wave = data[40].toInt();
  eg2_attack = data[41].toInt();
  eg2_decay = data[42].toInt();
  eg2_sustain = data[43].toInt();
  eg2_release = data[44].toInt();
  eg2_key_follow = data[45].toInt();

  updateosc1_sub();
  updateosc1_octave();
  updateosc1_wave();
  updateosc2_wave();
  updateosc2_xmod();
  updateosc2_eg_select();
  updatelfo1_wave();
  updatelfo2_wave();
  updatelfo3_wave();
  updatelfo_select();
  updateeg_select();

  //Patchname
  updatePatchname();

  Serial.print("Set Patch: ");
  Serial.println(patchName);
  if (updateParams) {
    sendToSynthData();
  }
}

void sendToSynthData() {

  // updateosc1_octave();
  // updateosc1_waveform();
  updateosc1_level();
  // updateosc2_octave();
  // updateosc2_waveform();
  updateosc2_level();
  // updateosc2_interval();
  // updateosc2_detune();
  // updatenoise();
  // updatevcf_cutoff();
  // updatevcf_res();
  // updatevcf_kbdtrack();
  // updatevcf_polarity();
  // updatevcf_eg_intensity();
  // updatechorus();
  // updatevcf_attack();
  // updatevcf_decay();
  // updatevcf_breakpoint();
  // updatevcf_slope();
  // updatevcf_sustain();
  // updatevcf_release();
  // updatevca_attack();
  // updatevca_decay();
  // updatevca_breakpoint();
  // updatevca_slope();
  // updatevca_sustain();
  // updatevca_release();
  // updatemg_frequency();
  // updatemg_delay();
  // updatemg_osc();
  // updatemg_vcf();
  // updatebend_osc();
  // updatebend_vcf();
  // updateglide_time();
  // updatePoly1();
  // updatePoly2();
  // updateUnison();
  // updatewaveBank();
}


void sendToSynth(int row) {

  // updateosc1_octave();
  // updateosc1_waveform();
  updateosc1_level();
  // updateosc2_octave();
  // updateosc2_waveform();
  updateosc2_level();
  // updateosc2_interval();
  // updateosc2_detune();
  // updatenoise();
  // updatevcf_cutoff();
  // updatevcf_res();
  // updatevcf_kbdtrack();
  // updatevcf_polarity();
  // updatevcf_eg_intensity();
  // updatechorus();
  // updatevcf_attack();
  // updatevcf_decay();
  // updatevcf_breakpoint();
  // updatevcf_slope();
  // updatevcf_sustain();
  // updatevcf_release();
  // updatevca_attack();
  // updatevca_decay();
  // updatevca_breakpoint();
  // updatevca_slope();
  // updatevca_sustain();
  // updatevca_release();
  // updatemg_frequency();
  // updatemg_delay();
  // updatemg_osc();
  // updatemg_vcf();
  // updatebend_osc();
  // updatebend_vcf();
  // updateglide_time();
  // updatePoly1();
  // updatePoly2();
  // updateUnison();
  // updatewaveBank();

  // Serial.print("Update Params ");
  // Serial.println(updateParams);
  if (!updateParams) {
    delay(2);
    writeRequest[5] = row;
    MIDI.sendSysEx(sizeof(writeRequest), writeRequest);
  }
}

String getCurrentPatchData() {
  return patchName + "," + String(osc1_octave) + "," + String(osc1_wave) + "," + String(osc1_pw) + "," + String(osc1_pwm) + "," + String(osc1_sub) + "," + String(osc1_level)
         + "," + String(osc2_freq) + "," + String(osc2_detune) + "," + String(osc2_wave) + "," + String(osc2_xmod) + "," + String(osc2_eg_depth) + "," + String(osc2_eg_select) + "," + String(osc2_level)
         + "," + String(vcf_cutoff) + "," + String(vcf_res) + "," + String(vcf_eg_depth) + "," + String(vcf_key_follow) + "," + String(vcf_key_velocity) + "," + String(vcf_hpf)
         + "," + String(lfo1_depth) + "," + String(lfo1_speed) + "," + String(lfo1_delay) + "," + String(lfo1_wave) + "," + String(lfo_select)
         + "," + String(eg1_attack) + "," + String(eg1_decay) + "," + String(eg1_sustain) + "," + String(eg1_release) + "," + String(eg1_key_follow) + "," + String(eg_select)
         + "," + String(vca_key_velocity) + "," + String(vca_level)
         + "," + String(lfo2_depth) + "," + String(lfo2_speed) + "," + String(lfo2_delay) + "," + String(lfo2_wave)
         + "," + String(lfo3_depth) + "," + String(lfo3_speed) + "," + String(lfo3_delay) + "," + String(lfo3_wave)
         + "," + String(eg2_attack) + "," + String(eg2_decay) + "," + String(eg2_sustain) + "," + String(eg2_release) + "," + String(eg2_key_follow);
}

void checkMux() {

  mux1Read = adc->adc1->analogRead(MUX1_S);

  if (mux1Read > (mux1ValuesPrev[muxInput] + QUANTISE_FACTOR) || mux1Read < (mux1ValuesPrev[muxInput] - QUANTISE_FACTOR)) {
    mux1ValuesPrev[muxInput] = mux1Read;
    mux1Read = (mux1Read >> resolutionFrig);  // Change range to 0-127

    switch (muxInput) {
      case MUX1_osc1_PW:
        myControlChange(midiChannel, CCosc1_PW, mux1Read);
        break;
      case MUX1_osc2_freq:
        myControlChange(midiChannel, CCosc2_freq, mux1Read);
        break;
      case MUX1_osc2_eg_depth:
        myControlChange(midiChannel, CCosc2_eg_depth, mux1Read);
        break;
      case MUX1_osc1_PWM:
        myControlChange(midiChannel, CCosc1_PWM, mux1Read);
        break;
      case MUX1_osc1_level:
        myControlChange(midiChannel, CCosc1_level, mux1Read);
        break;
      case MUX1_osc2_detune:
        myControlChange(midiChannel, CCosc2_detune, mux1Read);
        break;
      case MUX1_osc2_level:
        myControlChange(midiChannel, CCosc2_level, mux1Read);
        break;
    }
  }

  mux2Read = adc->adc1->analogRead(MUX2_S);

  if (mux2Read > (mux2ValuesPrev[muxInput] + QUANTISE_FACTOR) || mux2Read < (mux2ValuesPrev[muxInput] - QUANTISE_FACTOR)) {
    mux2ValuesPrev[muxInput] = mux2Read;
    mux2Read = (mux2Read >> resolutionFrig);  // Change range to 0-127

    switch (muxInput) {
      case MUX2_vcf_cutoff:
        myControlChange(midiChannel, CCvcf_cutoff, mux2Read);
        break;
      case MUX2_vcf_res:
        myControlChange(midiChannel, CCvcf_res, mux2Read);
        break;
      case MUX2_lfo1_depth:
        myControlChange(midiChannel, CClfo1_depth, mux2Read);
        break;
      case MUX2_lfo1_speed:
        myControlChange(midiChannel, CClfo1_speed, mux2Read);
        break;
      case MUX2_vcf_eg_depth:
        myControlChange(midiChannel, CCvcf_eg_depth, mux2Read);
        break;
      case MUX2_vcf_key_follow:
        myControlChange(midiChannel, CCvcf_key_follow, mux2Read);
        break;
      case MUX2_lfo2_depth:
        myControlChange(midiChannel, CClfo2_depth, mux2Read);
        break;
      case MUX2_lfo2_speed:
        myControlChange(midiChannel, CClfo2_speed, mux2Read);
        break;
    }
  }


  mux3Read = adc->adc1->analogRead(MUX3_S);

  if (mux3Read > (mux3ValuesPrev[muxInput] + QUANTISE_FACTOR) || mux3Read < (mux3ValuesPrev[muxInput] - QUANTISE_FACTOR)) {
    mux3ValuesPrev[muxInput] = mux3Read;
    mux3Read = (mux3Read >> resolutionFrig);  // Change range to 0-127

    switch (muxInput) {
      case MUX3_lfo1_delay:
        myControlChange(midiChannel, CClfo1_delay, mux3Read);
        break;
      case MUX3_lfo2_delay:
        myControlChange(midiChannel, CClfo2_delay, mux3Read);
        break;
      case MUX3_lfo3_delay:
        myControlChange(midiChannel, CClfo3_delay, mux3Read);
        break;


      case MUX3_vcf_key_velocity:
        myControlChange(midiChannel, CCvcf_key_velocity, mux3Read);
        break;
      case MUX3_vcf_hpf:
        myControlChange(midiChannel, CCvcf_hpf, mux3Read);
        break;
      case MUX3_lfo3_depth:
        myControlChange(midiChannel, CClfo3_depth, mux3Read);
        break;
      case MUX3_lfo3_speed:
        myControlChange(midiChannel, CClfo3_speed, mux3Read);
        break;
    }
  }

  mux4Read = adc->adc1->analogRead(MUX4_S);

  if (mux4Read > (mux4ValuesPrev[muxInput] + QUANTISE_FACTOR) || mux4Read < (mux4ValuesPrev[muxInput] - QUANTISE_FACTOR)) {
    mux4ValuesPrev[muxInput] = mux4Read;
    mux4Read = (mux4Read >> resolutionFrig);  // Change range to 0-127

    switch (muxInput) {
      case MUX4_eg1_attack:
        myControlChange(midiChannel, CCeg1_attack, mux4Read);
        break;
      case MUX4_eg1_decay:
        myControlChange(midiChannel, CCeg1_decay, mux4Read);
        break;
      case MUX4_eg1_sustain:
        myControlChange(midiChannel, CCeg1_sustain, mux4Read);
        break;
      case MUX4_eg1_release:
        myControlChange(midiChannel, CCeg1_release, mux4Read);
        break;
      case MUX4_eg2_attack:
        myControlChange(midiChannel, CCeg2_attack, mux4Read);
        break;
      case MUX4_eg2_decay:
        myControlChange(midiChannel, CCeg2_decay, mux4Read);
        break;
      case MUX4_eg2_sustain:
        myControlChange(midiChannel, CCeg2_sustain, mux4Read);
        break;
      case MUX4_eg2_release:
        myControlChange(midiChannel, CCeg2_release, mux4Read);
        break;
    }
  }

  mux5Read = adc->adc1->analogRead(MUX5_S);

  if (mux5Read > (mux5ValuesPrev[muxInput] + QUANTISE_FACTOR) || mux5Read < (mux5ValuesPrev[muxInput] - QUANTISE_FACTOR)) {
    mux5ValuesPrev[muxInput] = mux5Read;
    mux5Read = (mux5Read >> resolutionFrig);  // Change range to 0-127

    switch (muxInput) {
      case MUX5_eg1_key_follow:
        myControlChange(midiChannel, CCeg1_key_follow, mux5Read);
        break;
      case MUX5_vca_key_velocity:
        myControlChange(midiChannel, CCvca_key_velocity, mux5Read);
        break;
      case MUX5_vca_level:
        myControlChange(midiChannel, CCvca_level, mux5Read);
        break;
      case MUX5_eg2_key_follow:
        myControlChange(midiChannel, CCeg2_key_follow, mux5Read);
        break;
    }
  }

  muxInput++;
  if (muxInput >= MUXCHANNELS)
    muxInput = 0;

  digitalWriteFast(MUX_0, muxInput & B0001);
  digitalWriteFast(MUX_1, muxInput & B0010);
  digitalWriteFast(MUX_2, muxInput & B0100);
  delayMicroseconds(75);
}

void showSettingsPage() {
  showSettingsPage(settings::current_setting(), settings::current_setting_value(), state);
}

void midiCCOut(byte cc, byte value) {
  MIDI.sendControlChange(cc, value, midiChannel);  //MIDI DIN is set to Out
}

void checkSwitches() {

  saveButton.update();
  if (saveButton.held()) {
    switch (state) {
      case PARAMETER:
      case PATCH:
        state = DELETE;
        break;
    }
  } else if (saveButton.numClicks() == 1) {
    switch (state) {
      case PARAMETER:
        if (patches.size() < PATCHES_LIMIT) {
          resetPatchesOrdering();  //Reset order of patches from first patch
          patches.push({ patches.size() + 1, INITPATCHNAME });
          state = SAVE;
        }
        break;
      case SAVE:
        //Save as new patch with INITIALPATCH name or overwrite existing keeping name - bypassing patch renaming
        patchName = patches.last().patchName;
        state = PATCH;
        savePatch(String(patches.last().patchNo).c_str(), getCurrentPatchData());
        showPatchPage(patches.last().patchNo, patches.last().patchName);
        patchNo = patches.last().patchNo;
        loadPatches();  //Get rid of pushed patch if it wasn't saved
        setPatchesOrdering(patchNo);
        renamedPatch = "";
        state = PARAMETER;
        break;
      case PATCHNAMING:
        if (renamedPatch.length() > 0) patchName = renamedPatch;  //Prevent empty strings
        state = PATCH;
        savePatch(String(patches.last().patchNo).c_str(), getCurrentPatchData());
        showPatchPage(patches.last().patchNo, patchName);
        patchNo = patches.last().patchNo;
        loadPatches();  //Get rid of pushed patch if it wasn't saved
        setPatchesOrdering(patchNo);
        renamedPatch = "";
        state = PARAMETER;
        break;
    }
  }

  settingsButton.update();
  if (settingsButton.held()) {
    //If recall held, set current patch to match current hardware state
    //Reinitialise all hardware values to force them to be re-read if different
    state = REINITIALISE;
    reinitialiseToPanel();
  } else if (settingsButton.numClicks() == 1) {
    switch (state) {
      case PARAMETER:
        state = SETTINGS;
        showSettingsPage();
        break;
      case SETTINGS:
        showSettingsPage();
      case SETTINGSVALUE:
        settings::save_current_value();
        state = SETTINGS;
        showSettingsPage();
        break;
    }
  }

  backButton.update();
  if (backButton.held()) {
    //If Back button held, Panic - all notes off
  } else if (backButton.numClicks() == 1) {
    switch (state) {
      case RECALL:
        setPatchesOrdering(patchNo);
        state = PARAMETER;
        break;
      case SAVE:
        renamedPatch = "";
        state = PARAMETER;
        loadPatches();  //Remove patch that was to be saved
        setPatchesOrdering(patchNo);
        break;
      case PATCHNAMING:
        charIndex = 0;
        renamedPatch = "";
        state = SAVE;
        break;
      case DELETE:
        setPatchesOrdering(patchNo);
        state = PARAMETER;
        break;
      case SETTINGS:
        state = PARAMETER;
        break;
      case SETTINGSVALUE:
        state = SETTINGS;
        showSettingsPage();
        break;
    }
  }

  //Encoder switch
  recallButton.update();
  if (recallButton.held()) {
    //If Recall button held, return to current patch setting
    //which clears any changes made
    state = PATCH;
    //Recall the current patch
    patchNo = patches.first().patchNo;
    recallPatch(patchNo);
    state = PARAMETER;
  } else if (recallButton.numClicks() == 1) {
    switch (state) {
      case PARAMETER:
        state = RECALL;  //show patch list
        break;
      case RECALL:
        state = PATCH;
        //Recall the current patch
        patchNo = patches.first().patchNo;
        recallPatch(patchNo);
        state = PARAMETER;
        break;
      case SAVE:
        showRenamingPage(patches.last().patchName);
        patchName = patches.last().patchName;
        state = PATCHNAMING;
        break;
      case PATCHNAMING:
        if (renamedPatch.length() < 12)  //actually 12 chars
        {
          renamedPatch.concat(String(currentCharacter));
          charIndex = 0;
          currentCharacter = CHARACTERS[charIndex];
          showRenamingPage(renamedPatch);
        }
        break;
      case DELETE:
        //Don't delete final patch
        if (patches.size() > 1) {
          state = DELETEMSG;
          patchNo = patches.first().patchNo;     //PatchNo to delete from SD card
          patches.shift();                       //Remove patch from circular buffer
          deletePatch(String(patchNo).c_str());  //Delete from SD card
          loadPatches();                         //Repopulate circular buffer to start from lowest Patch No
          renumberPatchesOnSD();
          loadPatches();                      //Repopulate circular buffer again after delete
          patchNo = patches.first().patchNo;  //Go back to 1
          recallPatch(patchNo);               //Load first patch
        }
        state = PARAMETER;
        break;
      case SETTINGS:
        state = SETTINGSVALUE;
        showSettingsPage();
        break;
      case SETTINGSVALUE:
        settings::save_current_value();
        state = SETTINGS;
        showSettingsPage();
        break;
    }
  }
}

void reinitialiseToPanel() {
  //This sets the current patch to be the same as the current hardware panel state - all the pots
  //The four button controls stay the same state
  //This reinialises the previous hardware values to force a re-read
  muxInput = 0;
  for (int i = 0; i < MUXCHANNELS; i++) {
    mux1ValuesPrev[i] = RE_READ;
    mux2ValuesPrev[i] = RE_READ;
    mux3ValuesPrev[i] = RE_READ;
    mux4ValuesPrev[i] = RE_READ;
    mux5ValuesPrev[i] = RE_READ;
  }
  patchName = INITPATCHNAME;
  showPatchPage("Initial", "Panel Settings");
}

void checkEncoder() {
  //Encoder works with relative inc and dec values
  //Detent encoder goes up in 4 steps, hence +/-3

  long encRead = encoder.read();
  if ((encCW && encRead > encPrevious + 3) || (!encCW && encRead < encPrevious - 3)) {
    switch (state) {
      case PARAMETER:
        state = PATCH;
        patches.push(patches.shift());
        patchNo = patches.first().patchNo;
        recallPatch(patchNo);
        state = PARAMETER;
        break;
      case RECALL:
        patches.push(patches.shift());
        break;
      case SAVE:
        patches.push(patches.shift());
        break;
      case PATCHNAMING:
        if (charIndex == TOTALCHARS) charIndex = 0;  //Wrap around
        currentCharacter = CHARACTERS[charIndex++];
        showRenamingPage(renamedPatch + currentCharacter);
        break;
      case DELETE:
        patches.push(patches.shift());
        break;
      case SETTINGS:
        settings::increment_setting();
        showSettingsPage();
        break;
      case SETTINGSVALUE:
        settings::increment_setting_value();
        showSettingsPage();
        break;
    }
    encPrevious = encRead;
  } else if ((encCW && encRead < encPrevious - 3) || (!encCW && encRead > encPrevious + 3)) {
    switch (state) {
      case PARAMETER:
        state = PATCH;
        patches.unshift(patches.pop());
        patchNo = patches.first().patchNo;
        recallPatch(patchNo);
        state = PARAMETER;
        break;
      case RECALL:
        patches.unshift(patches.pop());
        break;
      case SAVE:
        patches.unshift(patches.pop());
        break;
      case PATCHNAMING:
        if (charIndex == -1)
          charIndex = TOTALCHARS - 1;
        currentCharacter = CHARACTERS[charIndex--];
        showRenamingPage(renamedPatch + currentCharacter);
        break;
      case DELETE:
        patches.unshift(patches.pop());
        break;
      case SETTINGS:
        settings::decrement_setting();
        showSettingsPage();
        break;
      case SETTINGSVALUE:
        settings::decrement_setting_value();
        showSettingsPage();
        break;
    }
    encPrevious = encRead;
  }
}

void SaveCurrent() {
  if (saveCurrent) {
    state = SETTINGS;
    if (midiOutCh > 0) {
      MIDI.sendSysEx(sizeof(saveRequest), saveRequest);
    }
    saveCurrent = false;
    storeSaveCurrent(saveCurrent);
    settings::decrement_setting_value();
    settings::save_current_value();
    showSettingsPage();
    delay(100);
    state = PARAMETER;
    //recallPatch(patchNo);
  }
}

void SaveAll() {
  if (saveAll) {
    state = SETTINGS;
    for (int row = 0; row < 64; row++) {
      if (midiOutCh > 0) {
        MIDI.sendProgramChange(row, midiOutCh);
        delay(10);
        MIDI.sendSysEx(sizeof(saveRequest), saveRequest);
        delay(10);
      }
    }
    saveAll = false;
    storeSaveAll(saveAll);
    settings::decrement_setting_value();
    settings::save_current_value();
    showSettingsPage();
    delay(100);
    state = PARAMETER;
    recallPatch(patchNo);
  }
}

void checkLoadFactory() {
  if (loadFactory) {
    for (int row = 0; row < 64; row++) {
      String currentRow = factory[row];

      String values[39];   // Assuming you have 38 values per row
      int valueIndex = 0;  // Index for storing values
      for (unsigned int i = 0; i < currentRow.length(); i++) {
        char currentChar = currentRow.charAt(i);

        // Check for the delimiter (",") and move to the next value
        if (currentChar == ',') {
          valueIndex++;  // Move to the next value
          continue;      // Skip the delimiter
        }

        // Append the character to the current value
        values[valueIndex] += currentChar;
      }

      // Process the values
      int intValues[39];
      for (int i = 0; i < 39; i++) {  // Adjust the loop count based on the number of values per row
        switch (i) {

          // case 0:
          //   patchName = values[i];
          //   break;

          // case 1:
          //   intValues[i] = values[i].toInt();
          //   switch (intValues[i]) {
          //     case 16:
          //       osc1_octave = 0;
          //       break;
          //     case 8:
          //       osc1_octave = 1;
          //       break;
          //     case 4:
          //       osc1_octave = 2;
          //       break;
          //   }
          //   break;

          // case 2:
          //   intValues[i] = values[i].toInt();
          //   osc1_waveform = (intValues[i] - 1);
          //   break;

          // case 3:  // osc1_level
          //   intValues[i] = values[i].toInt();
          //   osc1_level = intValues[i];
          //   break;

          // case 4:
          //   intValues[i] = values[i].toInt();
          //   switch (intValues[i]) {
          //     case 16:
          //       osc2_octave = 0;
          //       break;
          //     case 8:
          //       osc2_octave = 1;
          //       break;
          //     case 4:
          //       osc2_octave = 2;
          //       break;
          //   }
          //   break;

          // case 5:
          //   intValues[i] = values[i].toInt();
          //   osc2_waveform = (intValues[i] - 1);
          //   break;

          // case 6:  // osc2_level
          //   intValues[i] = values[i].toInt();
          //   osc2_level = intValues[i];
          //   break;

          // case 7:  // osc2_interval
          //   intValues[i] = values[i].toInt();
          //   switch (intValues[i]) {
          //     case 1:
          //       osc2_interval = 0;
          //       break;
          //     case 3:
          //       osc2_interval = 1;
          //       break;
          //     case -3:
          //       osc2_interval = 2;
          //       break;
          //     case 4:
          //       osc2_interval = 3;
          //       break;
          //     case 5:
          //       osc2_interval = 4;
          //       break;
          //   }
          //   break;

          // case 8:  // osc2_detune
          //   intValues[i] = values[i].toInt();
          //   osc2_detune = intValues[i];
          //   break;

          // case 9:  // moise_level
          //   intValues[i] = values[i].toInt();
          //   noise = intValues[i];
          //   break;

          // case 10:  // cutoff
          //   intValues[i] = values[i].toInt();
          //   vcf_cutoff = intValues[i];
          //   break;

          // case 11:  // res
          //   intValues[i] = values[i].toInt();
          //   vcf_res = intValues[i];
          //   break;

          // case 12:  // kbdtrack
          //   intValues[i] = values[i].toInt();
          //   vcf_kbdtrack = intValues[i];
          //   break;

          // case 13:  // polarity
          //   intValues[i] = values[i].toInt();
          //   vcf_polarity = (intValues[i] - 1);
          //   break;

          // case 14:  // eg_intensity
          //   intValues[i] = values[i].toInt();
          //   vcf_eg_intensity = intValues[i];
          //   break;

          // case 15:  // chorus
          //   intValues[i] = values[i].toInt();
          //   chorus = intValues[i];
          //   break;

          // case 16:  // vcf_attack
          //   intValues[i] = values[i].toInt();
          //   vcf_attack = intValues[i];
          //   break;

          // case 17:  // vcf_decay
          //   intValues[i] = values[i].toInt();
          //   vcf_decay = intValues[i];
          //   break;

          // case 18:  // vcf_bp
          //   intValues[i] = values[i].toInt();
          //   vcf_breakpoint = intValues[i];
          //   break;

          // case 19:  // vcf_slope
          //   intValues[i] = values[i].toInt();
          //   vcf_slope = intValues[i];
          //   break;

          // case 20:  // vcf_sustain
          //   intValues[i] = values[i].toInt();
          //   vcf_sustain = intValues[i];
          //   break;

          // case 21:  // vcf_release
          //   intValues[i] = values[i].toInt();
          //   vcf_release = intValues[i];
          //   break;

          // case 22:  // vca_attack
          //   intValues[i] = values[i].toInt();
          //   vca_attack = intValues[i];
          //   break;

          // case 23:  // vca_decay
          //   intValues[i] = values[i].toInt();
          //   vca_decay = intValues[i];
          //   break;

          // case 24:  // vca_bp
          //   intValues[i] = values[i].toInt();
          //   vca_breakpoint = intValues[i];
          //   break;

          // case 25:  // vca_slope
          //   intValues[i] = values[i].toInt();
          //   vca_slope = intValues[i];
          //   break;

          // case 26:  // vca_sustain
          //   intValues[i] = values[i].toInt();
          //   vca_sustain = intValues[i];
          //   break;

          // case 27:  // vca_release
          //   intValues[i] = values[i].toInt();
          //   vca_release = intValues[i];
          //   break;

          // case 28:  // mg_freq
          //   intValues[i] = values[i].toInt();
          //   mg_frequency = intValues[i];
          //   break;

          // case 29:  // mg_delay
          //   intValues[i] = values[i].toInt();
          //   mg_delay = intValues[i];
          //   break;

          // case 30:  // mg_osc
          //   intValues[i] = values[i].toInt();
          //   mg_osc = intValues[i];
          //   break;

          // case 31:  // mg_vcf
          //   intValues[i] = values[i].toInt();
          //   mg_vcf = intValues[i];
          //   break;

          // case 32:  // bend_osc
          //   intValues[i] = values[i].toInt();
          //   bend_osc = intValues[i];
          //   break;

          // case 33:  // bend_vcf
          //   intValues[i] = values[i].toInt();
          //   bend_vcf = intValues[i];
          //   break;

          // case 34:  // glide
          //   intValues[i] = values[i].toInt();
          //   glide_time = intValues[i];
          //   break;

          // case 35:  // poly1
          //   intValues[i] = values[i].toInt();
          //   poly1 = intValues[i];
          //   break;

          // case 36:  // poly2
          //   intValues[i] = values[i].toInt();
          //   poly2 = intValues[i];
          //   break;

          // case 37:  // unison
          //   intValues[i] = values[i].toInt();
          //   unison = intValues[i];
          //   break;

          // case 38:  // wave_bank
          //   intValues[i] = values[i].toInt();
          //   wave_bank = intValues[i];
          //   break;
        }
      }
      // Add a newline to separate rows (optional)
      sprintf(buffer, "%d", row + 1);
      savePatch(buffer, getCurrentPatchData());
      updatePatchname();
      recallPatchFlag = true;
      sendToSynth(row);
      delay(2);
      writeRequest[5] = row;
      MIDI.sendSysEx(sizeof(writeRequest), writeRequest);
      recallPatchFlag = false;
    }
    loadPatches();
    loadFactory = false;
    storeLoadFactory(loadFactory);
    settings::decrement_setting_value();
    settings::save_current_value();
    showSettingsPage();
    delay(100);
    state = PARAMETER;
    recallPatch(1);
    MIDI.sendProgramChange(0, midiOutCh);
  }
}

void loop() {
  octoswitch.update();
  srp.update();  // update all the LEDs in the buttons
  checkMux();
  checkSwitches();
  checkEncoder();
  MIDI.read(midiChannel);
  usbMIDI.read(midiChannel);
  checkLoadFromDW();
  checkLoadFactory();
  SaveCurrent();
  SaveAll();
}
