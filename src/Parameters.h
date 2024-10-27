//Values below are just for initialising and will be changed when synth is initialised to current panel controls & EEPROM settings
byte midiChannel = MIDI_CHANNEL_OMNI;  //(EEPROM)
byte midiOutCh = 1;                    //(EEPROM)

int readresdivider = 32;
int resolutionFrig = 5;

char buffer[10];

int MIDIThru = midi::Thru::Off;  //(EEPROM)
String patchName = INITPATCHNAME;
String currentRow;
String bankdir = "/Bank";
boolean encCW = true;  //This is to set the encoder to increment when turned CW - Settings Option
boolean recallPatchFlag = true;
boolean loadFactory = false;
boolean loadRAM = false;
boolean loadFromDW = false;
boolean ROMType = false;
boolean dataInProgress = false;
int currentSendPatch = 0;
boolean saveCurrent = false;
boolean saveAll = false;
boolean updateParams = false;  //(EEPROM)
int bankselect = 0;
int old_value = 0;
int old_param_offset = 0;
int received_patch = 0;


int osc1_octave = 0;
int osc1_wave = 0;
int osc1_pw = 0;
int osc1_pwm = 0;
int osc1_sub = 0;
int osc1_level = 0;

int osc2_freq = 0;
int osc2_detune = 0;
int osc2_wave = 0;
int osc2_xmod = 0;
int osc2_eg_depth = 0;
int osc2_eg_select = 0;
int osc2_level = 0;

int vcf_cutoff = 0;
int vcf_res = 0;
int vcf_eg_depth = 0;
int vcf_key_follow = 0;
int vcf_key_velocity = 0;
int vcf_hpf = 0;

int lfo1_depth = 0;
int lfo1_speed = 0;
int lfo1_delay = 0;
int lfo1_wave = 0;
int lfo_select = 0;

int eg1_attack = 0;
int eg1_decay = 0;
int eg1_sustain = 0;
int eg1_release = 0;
int eg1_key_follow = 0;
int eg_select = 0;

int vca_key_velocity = 0;
int vca_level = 0;

int lfo2_depth = 0;
int lfo2_speed = 0;
int lfo2_delay = 0;
int lfo2_wave = 0;

int lfo3_depth = 0;
int lfo3_speed = 0;
int lfo3_delay = 0;
int lfo3_wave = 0;

int eg2_attack = 0;
int eg2_decay = 0;
int eg2_sustain = 0;
int eg2_release = 0;
int eg2_key_follow = 0;

int lfo_select_temp = -1;
int eg_select_temp = -1;

int returnvalue = 0;

//Pick-up - Experimental feature
//Control will only start changing when the Knob/MIDI control reaches the current parameter value
//Prevents jumps in value when the patch parameter and control are different values
boolean pickUp = false;  //settings option (EEPROM)
boolean pickUpActive = false;
#define TOLERANCE 2  //Gives a window of when pick-up occurs, this is due to the speed of control changing and Mux reading

// byte byteArray[0] = 0xF0;  // Start of SysEx
// byte byteArray[1] = 0x47;  // Manufacturer ID (example value)
// byte byteArray[2] = 0x7E;  // Data byte 1 (example value)
// byte byteArray[3] = 0x04;  // program (example value)
// byte byteArray[4] = 0x00;
// byte byteArray[5] = 0x00;
// byte byteArray[6] = 0x00;
// byte byteArray[7] = 0x00;
// byte byteArray[8] = 0x00;
// byte byteArray[9] = 0x00;
// byte byteArray[10] = 0x00;
// byte byteArray[11] = 0x00;
// byte byteArray[12] = 0x00;
// byte byteArray[13] = 0x00;
// byte byteArray[14] = 0x00;
// byte byteArray[15] = 0x00;
// byte byteArray[16] = 0x00;
// byte byteArray[17] = 0x00;
// byte byteArray[18] = 0x00;
// byte byteArray[19] = 0x00;
// byte byteArray[20] = 0x00;
// byte byteArray[21] = 0x00;
// byte byteArray[22] = 0x00;
// byte byteArray[23] = 0x00;
// byte byteArray[24] = 0x00;
// byte byteArray[25] = 0x00;
// byte byteArray[26] = 0x00;
// byte byteArray[27] = 0x00;
// byte byteArray[28] = 0x00;
// byte byteArray[29] = 0x00;
// byte byteArray[30] = 0x00;
// byte byteArray[31] = 0x00;
// byte byteArray[32] = 0x00;
// byte byteArray[33] = 0x00;
// byte byteArray[34] = 0x00;
// byte byteArray[35] = 0x00;
// byte byteArray[36] = 0x00;
// byte byteArray[37] = 0x00;
// byte byteArray[38] = 0x00;
// byte byteArray[39] = 0x00;
// byte byteArray[40] = 0x00;
// byte byteArray[41] = 0x00;
// byte byteArray[42] = 0x00;
// byte byteArray[43] = 0x00;
// byte byteArray[44] = 0x00;
// byte byteArray[45] = 0x00;
// byte byteArray[46] = 0x00;
// byte byteArray[47] = 0x00;
// byte byteArray[48] = 0x00;
// byte byteArray[49] = 0xF7;  // End of Exclusive
