#include <avr/pgmspace>

typedef struct {
    prog_uchar reg_off;
    prog_uchar reg_val;
} reg_value;

typedef struct {
            uint8_t control_page;           //coefficient page location
            uint8_t control_base;           //coefficient base address within page
            uint8_t control_mute_flag;      //non-zero means muting required
            uint8_t control_string_index;   //string table index
} control; 

static control MUX_controls[] = {
//{48,52,1,0}
//{44,32,1,0}
//{9,8,1,0}
//{44,100,1,0}
{8,28,1,0},
{11,56,1,1},
{45,68,1,2}
};

static control VOLUME_controls[] = {
//{48,116,0,0}
//{8,48,0,0}
//{44,104,0,0}
//{44,48,0,0}
//{44,28,0,0}
{44,28,0,0}
};


static control DRC_controls[] = {
//{48,116,0,0}
//{8,48,0,0}
//{44,104,0,0}
//{44,48,0,0}
{44,72,0,0}

};


reg_value diagnosis[] = {
{  0,0x00},  //go to page 0
{44, 0},  //Page 0 / Register 44: Sticky Flag Register 2 - 0x00 / 0x2C - Heaphone over current flag
{0, 0x01},  //go to page 1
{2, 0},    //Page 1 / Register 2: LDO Control Register - 0x01 / 0x02 - LDO over current flag
};

const reg_value REG_Section_program[] PROGMEM= {
    {  0,0x00},
//			# reg[  0][  1] = 0x01	; Initialize the device through software reset
    {  1,0x01},
                        //Device start up lockout time after the reset
    {254, 0x02},        //direct the PCM3070_conf() function to wait 2ms
    {  0,0x01},
//			# reg[  1][  1] = 0x08	; Disable weak AVDD to DVDD connection
    {  1,0x08},
//			# reg[  1][  2] = 0x00	; Enable Master Analog Power Control
    {  2,0b00000001},
//			# reg[  1][ 71] = 0x32	; Set the input power-up time to 3.1ms
    { 71,0x32},
//			# reg[  1][123] = 0x05	; Force REF charging time to 40ms
    {123,0x05},
    {254, 40},          //delay for REF
    {255,0x00},        //direct the PCM3070_conf() function to program miniDSPs
    //{255,0x01},
    {  0,0x00},
//			# reg[  0][ 60] = 0x00	; Use miniDSP_D for signal processing
    { 60,0b11000000}, //- miniDSP_A and miniDSP_D are powered up together. Useful when there is data transfer between miniDSP_A and miniDSP_D
//			# reg[  0][ 61] = 0x00	; Use miniDSP_A for signal processing
    { 61,0x00},
    //{ 62,0b00010000},      //Do not reset miniDSP_A instruction counter at the start of new frame.
  
    { 17,0b00001000},       //reg[  0][ 17] = 0x08	; 8x Interpolation                        
    { 23,0b00000100},   //Register 23: miniDSP_A Decimation Factor Setting Register - 0x00 / 0x17
                  
    { 15,0x03},
//			
    { 16,0x80},
//			
    { 21,0x03},
//			
    { 22,0x80},
	
    {  0,0x08},
//			# reg[  8][  1] = 0x04	; adaptive mode for ADC
    {  1,0x04},
    {  0,0x2C},
//			# reg[ 44][  1] = 0x04	; adaptive mode for DAC
    {  1,0x04},
    {  0,0x00},
//			# reg[  0][  5] = 0x91	; P=1, R=3
    {  5,0b10010011},
//			# reg[  0][  6] = 0x08	; J=4
    {  6,0b00000100},
//			# reg[  0][  7] = 0x00	; D=0000 (MSB)
    {  7,0b00000000},
//			# reg[  0][  8] = 0x00	; D=0000 (LSB)
    {  8,0b00000000},
//			# reg[  0][  4] = 	; PLL_clkin = MCLK, codec_clkin = PLL_CLK, PLL on
    {  4,0b00000011},
	
//			# reg[  0][254] = 0x0a	; Delay 10ms for PLL to lock
    {254,0x0A},          //direct the PCM3070_conf() function to wait 10ms
    
//			# reg[  0][ 12] = 0x88	; MDAC = 8, divider powered on
    { 12,0b10001000},
//			# reg[  0][ 13] = 0x00	; DOSR = 128 (MSB)
    { 13,0b00000000},
//			# reg[  0][ 14] = 0x80	; DOSR = 128 (LSB)
    { 14,0b10000000},
//			# reg[  0][ 18] = 0x02	; NADC = 2, divider powered on
    { 18,0b10000010},
//			# reg[  0][ 19] = 0x88	; MADC = 8, divider powered on
    { 19,0b10001000},
//			# reg[  0][ 20] = 0x80	; AOSR = 128
    { 20,0b10000000},    
//			# reg[  0][ 11] = 0x82	; NDAC = 2, divider powered on
    { 11,0b10000010},     

  
  
    {27, 0b00101100},    //output bclk, wclk, data word length 24
    {29, 0b00000011},    //BDIV_CLKIN = ADC_MOD_CLK
    {30, 0b10000010},    //BCLK N divider = 4
    
    {25, 0b00000111},      // CDIV_CLKIN=ADC_MOD_CLK
    {26, 0b10000001},    //CLKOUT enable divider, M = 1
    {55, 0b00000110},     //MISO is CLKOUT output
    //{53, 0b00000010},    //DOUT Bus Keeper Enabled, DOUT is Primary DOUT (Default)
    
    
    {  0,0x01},        

//			# reg[  1][ 51] = 0x40	; Mic Bias enabled, Source = Avdd, 1.25V
    { 51,0x40},
//			# reg[  1][ 52] = 0x40	; Route IN1L to LEFT_P with 10K input impedance
    { 52,0x40},
//			# reg[  1][ 54] = 0x40	; Route CM1L to LEFT_M with 10K input impedance
    { 54,0x40},
//			# reg[  1][ 55] = 0x40	; Route IN1R to RIGHT_P with 10K input impedance
    { 55,0x40},
//			# reg[  1][ 57] = 0x40	; Route CM1R to RIGHT_M with 10K input impedance
    { 57,0x40},

	
//			# reg[  1][ 59] = 0x00	; Enable MicPGA_L Gain Control, 0dB
    { 59,0x06},	//0x07 is 3.5db which is the maximum amount of gain that we can apply before the ADC saturates at 4Vpp input. 4Vpp is the highest voltage I found from a music source (It was a MacBook Pro Retina)
//			# reg[  1][ 60] = 0x00	; Enable MicPGA_R Gain Control, 0dB
    { 60,0x06},
    {  0,0x00},
//			# reg[  0][ 81] = 0xc0	; Power up LADC/RADC
    { 81,0xC0},
//			# reg[  0][ 82] = 0x00	; Unmute LADC/RADC
    { 82,0x00},
    {  0,0x01},
//			# reg[  1][ 20] = 0x25	; De-pop: 5 time constants, 6k resistance
    { 20,0x25},
//			# reg[  1][ 12] = 0x08	; Route LDAC to HPL
    { 12,0b00001000},
//			# reg[  1][ 13] = 0x08	; Route RDAC to HPR
    { 13,0b00001000},
//			# reg[  1][ 14] = 0x08	; Route LDAC to LOL
    { 14,0x08},
//			# reg[  1][ 15] = 0x08	; Route LDAC to LOR
    { 15,0x08},
//			# reg[  1][ 10] = 	; Common Mode Control Register
    { 10,0b00111011}, 
    
    {  0,0x00},
//			# reg[  0][ 63] = 0xd4	; Power up LDAC/RDAC w/ soft stepping
    { 63,0xD5},
    {  0,0x01},
	
	{22, 0b00100110},	//set initial HPL volume to ~-20db
	{23, 0b00100110},	//set initial HPR volume

//			# reg[  1][ 16] = 0x00	; Unmute HPL driver, 0dB Gain
//    { 16,0b00010000},
//			# reg[  1][ 17] = 0x00	; Unmute HPR driver, 0dB Gain
//   { 17,0b00010000},
//			# reg[  1][ 18] = 0x00	; Unmute LOL driver, dB Gain
    { 18,0b00010101},
//			# reg[  1][ 19] = 0x00	; Unmute LOR driver, dB Gain
    { 19,0b00010101},
//			# reg[  1][  9] = 0x3c	; Power up HPL/HPR, MAL, MAR, and LOL/LOR drivers
    {  9,0b00111111},
    {  0,0x00},
//			# reg[  0][ 64] = 0x00	; Unmute LDAC/RDAC

    { 64,0x00},
    
//			# reg[0][82] = 0
    { 82,0x00},
//			# reg[0][83] = 0
    { 83,0x00},
//			# reg[0][86] = 128
    { 86,0x00},
//			# reg[0][87] = 62
    { 87,0x3E},
//			# reg[0][88] = 12	- gain
    { 88,0x0C},
//			# reg[0][89] = 104
    { 89,0x68},
//			# reg[0][90] = 168
    { 90,0xA8},
//			# reg[0][91] = 6
    { 91,0x06},
//			# reg[0][92] = 0
    { 92,0x00},
//			# reg[0][84] = 0
    { 84,0x00},
//			# reg[0][94] = 128
    { 94,0x00},
//			# reg[0][95] = 62
    { 95,0x3E},
//			# reg[0][96] = 12	- gain
    { 96,0x0C},
//			# reg[0][97] = 104
    { 97,0x68},
//			# reg[0][98] = 168
    { 98,0xA8},
//			# reg[0][99] = 6
    { 99,0x06},
//			# reg[0][100] = 0
    {100,0x00},
};

const reg_value aux_input_config[] PROGMEM = {
    {  0,0x01},        

//			# reg[  1][ 51] = 0x40	; Mic Bias enabled, Source = Avdd, 1.25V
//    { 51,0x40},
//			# reg[  1][ 52] = 0x40	; Route IN1L to LEFT_P with 10K input impedance
    { 52,0x40},
//			# reg[  1][ 54] = 0x40	; Route CM1L to LEFT_M with 10K input impedance
    { 54,0x40},
//			# reg[  1][ 55] = 0x40	; Route IN1R to RIGHT_P with 10K input impedance
    { 55,0x40},
//			# reg[  1][ 57] = 0x40	; Route CM1R to RIGHT_M with 10K input impedance
    { 57,0x40},

	
//			# reg[  1][ 59] = 0x00	; Enable MicPGA_L Gain Control, 0dB
    { 59,0x06},	//0x07 is 3.5db which is the maximum amount of gain that we can apply before the ADC saturates at 4Vpp input. 4Vpp is the highest voltage I found from a music source (It was a MacBook Pro Retina)
//			# reg[  1][ 60] = 0x00	; Enable MicPGA_R Gain Control, 0dB
    { 60,0x06},
};
const reg_value phono_input_config[] PROGMEM = {
    {  0,0x01},        

//			# reg[  1][ 51] = 0x40	; Mic Bias enabled, Source = Avdd, 1.25V
   // { 51,0x40},
//			# reg[  1][ 52] = 0b00110000	; Route IN2L to LEFT_P with 10K input impedance
    { 52,0b00000100},
//			# reg[  1][ 54] = 0b00110000	; Route IN2R to CMM with 10K input impedance
    { 54,0b01000000},
//			# reg[  1][ 55] = 0b00001100	; Route IN3R to RIGHT_P with 10K input impedance
    { 55,0b00000100},
//			# reg[  1][ 57] = 0b00001100	; Route IN3L to CMM with 10K input impedance
    { 57,0b01000000},

	
//			# reg[  1][ 59] = 0x00	; Enable MicPGA_L Gain Control, 0dB
    { 59,0x46},	//0x07 is 3.5db which is the maximum amount of gain that we can apply before the ADC saturates at 4Vpp input. 4Vpp is the highest voltage I found from a music source (It was a MacBook Pro Retina)
//			# reg[  1][ 60] = 0x00	; Enable MicPGA_R Gain Control, 0dB
    { 60,0x46},
};
