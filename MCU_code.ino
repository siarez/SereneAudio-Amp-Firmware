#include <EEPROM.h>
#include <PinChangeInt.h>
#include <Wire.h>
//#include <stdio.h>
#include <avr/pgmspace>
#include <SoftPWM.h>
#include <SoftPWM_timer.h>
#include "REG_Section_Program.h"
#include "main_Rate48_pps_driver.h"
#include "TAS5708.h"

#define PCM3070_address  0b0011000
#define TAS5708_address  0b0011011


//variables used in volume increment-decrement 
volatile int encoder0PinA =4;	//PD4
volatile int encoder0PinB = 7;	//PD7
volatile int pb_rotation_flag=LOW;
volatile byte volume_change_direction; //LOW = volume decrease, HIGH = volume increase

//variables used in pushbutton interrupt 
volatile int pb_flag=LOW;
volatile int switched_input_flag=LOW;	//when input is switched this flag is set
volatile int selected_input=0;
volatile unsigned long pb_flag_timeStamp=0;
const unsigned long debounce_period=100;
const unsigned long press_and_hold_period=1000;
const int PB_pin = 3;	//PD3

const int LED_pin = A0;

const int TAS5708_reset_pin = A3;
const int TAS5708_PDN_pin = A2;
const int TAS5708_HIZ = 2; //PD2
const int PVCC_Mosfet = 10; //PB2

//variables used in sub-woofer jack in/0ut
volatile int sub_jack = 9;
volatile int subjack_flag=HIGH;

//variables used in headphone jack in/out
volatile int headjackIn_flag=LOW;
volatile byte mute_state = HIGH;
volatile int battVolts,battVolts_old;   // made global for wider avaliblity throughout a sketch if needed, example a low voltage alarm, etc
volatile byte LED_blink_flag = LOW;

volatile unsigned long dsp_vol_coef;
volatile unsigned long dsp_mute_vol = 0x00000F00ul;
//volatile byte amp_vol_coef;
volatile char amp_vol_db;
#define amp_vol_db_max  19	//This is the maximum gain that before the Vpp goes above 19Vpp at 0dBFS (No BiQuads).

volatile unsigned long time;

void setup() {
	// put your setup code here, to run once:
	//TAS5708 pin setup
	pinMode(PVCC_Mosfet, OUTPUT);
	digitalWrite(PVCC_Mosfet, LOW);
	pinMode(TAS5708_PDN_pin, OUTPUT);
	pinMode(TAS5708_HIZ, INPUT);  //set to input to use internal pull-up 	
	digitalWrite(TAS5708_PDN_pin, HIGH);
	digitalWrite(TAS5708_HIZ, HIGH );
	pinMode(TAS5708_reset_pin, OUTPUT);
	digitalWrite(TAS5708_reset_pin, LOW);
	
	delayMicroseconds(100);
	digitalWrite(PVCC_Mosfet, HIGH);
	
	
	
	pinMode(LED_pin, OUTPUT);
	// Initialize Soft PWM
	SoftPWMBegin();
	// Set fade time for LED_pin to 100 ms fade-up time, and 500 ms fade-down time
	SoftPWMSetFadeTime(LED_pin, 0, 0);	
	SoftPWMSetPercent(LED_pin, 100);	//red

	Serial.begin(9600);
	//Setup I2C
	Wire.begin(); 

	PCM3070_Conf();
	selected_input = 255 - EEPROM.read(0);	//read the selected input from EEPROM
	if(selected_input == 0)
	{
		switch_input();
		mux_select(2, 1);
	}
	else
	{
		switch_input();
		mux_select(1, 1);
	}	
	
	TAS5708_Conf();
	
	
	//setup for volume increment-decrement
	pinMode(encoder0PinA, INPUT); 
	pinMode(encoder0PinB, INPUT);
	digitalWrite(encoder0PinA, HIGH);
	digitalWrite(encoder0PinB, HIGH); 
	PCintPort::attachInterrupt(encoder0PinA, &volume_change_interrupt, RISING);

//setup for pushbutton interrupt
	pinMode(PB_pin, INPUT);
	digitalWrite(PB_pin, HIGH);       // turn on pullup resistors
	interrupts();
	attachInterrupt(1,pushbutton_interrupt,CHANGE); 


	//setup for sub-woofer jack sense
	pinMode(sub_jack, INPUT);
	PCintPort::attachInterrupt(sub_jack,subjack_interrupt,CHANGE);


	//setup for headphone jack sense
	pinMode(A1,INPUT);
	PCintPort::attachInterrupt(A1,headjackIn_interrupt,CHANGE);	
	
	//mute DSP after setup
	//mute_subwoofer();
	mute_amp();
	mute_headphone();
	mute_state = HIGH;  
	
	//initialize volumes
	//The maximum volume for the amp chip should be the 0db point for the subwoofer.
	//It should also be the maximum volume for the headphone
	write_headphone_volume_to_DSP(0b00100110);	//-20db
	dsp_vol_coef = 0x143D1300ul;		//0x0409C200 = -10dB, This is the subwoofer volume
	dsp_vol_coef = 0x0E53EB00ul;		//0x0409C200 = -13dB, This is the subwoofer volume
	write_volume_to_DSP(&dsp_vol_coef);				//-20db
	//amp_vol_coef = 0b01001100;			//0b01001100 = -14dB
/*
//I set the initial vol to a multiple of 2 minus max volume. 
This is to allow reach amp_vol_db_max when the volume is changed at 2db increments
*/
	amp_vol_db = amp_vol_db_max - 2*16;	
	write_volume_to_amp(amp_vol_db);
	
	
	Serial.println("Setup done");
	
	//battVolts=getBandgap();	
	//Serial.print("VCC: ");
	//Serial.println(battVolts);
    mute_amp();
	
	SoftPWMSetPercent(LED_pin, selected_input * 50);
	delay(150);
	SoftPWMSetPercent(LED_pin, 100);
	pb_flag=LOW;
}
volatile byte PB_state;
void loop() {

	time = millis();
	PB_state = digitalRead(PB_pin);
	//unmute_amp();
	//code for pushbutton interrupt**********************
	if ((pb_flag==HIGH)&&((time - pb_flag_timeStamp)>debounce_period))
	{
		if (switched_input_flag == HIGH)
		{	//If we have just switched input, we don't want to mute or unmute when user lets go of push button
			switched_input_flag = LOW;
			pb_flag=LOW;  
		}
		else if(PB_state == HIGH)
		{
			if(mute_state == LOW){	//It's not mute, so we mute it.			
				//mute_subwoofer();
				mute_amp();
				mute_headphone();				
				//digitalWrite(LED_pin, HIGH);	//turn LED red
				SoftPWMSetPercent(LED_pin, 100);
				Serial.println("mute");
				mute_state = HIGH;
			}
			else if(mute_state == HIGH){ //It is muted, so we unmute it
				if(digitalRead(A1)==LOW){	//headphone is not in
					//unmute_subwoofer();
					mux_select(1, 2);
					unmute_amp();
				}
				else {//headphone is in
					mux_select(2, 2);
					unmute_headphone();				
				}
				SoftPWMSetPercent(LED_pin, selected_input * 50);
				Serial.println("unmute");
				mute_state = LOW;
			}
			Serial.println("PB pressed");
			pb_flag=LOW;  
		} 
		else if((PB_state == LOW) && ((time - pb_flag_timeStamp) > press_and_hold_period))
		{
			//switch input and update EEPROM
			mute_amp();
			mute_headphone();
			delay(200);
			if(selected_input == 0)
			{
				selected_input = 1;
				switch_input();
				mux_select(1, 1);
			}
			else
			{
				selected_input = 0;
				switch_input();
				mux_select(2, 1);
			}
			delay(200);
			if(digitalRead(A1)==LOW){	//headphone is not in
				//unmute_subwoofer();
				mux_select(1, 2);
				unmute_amp();
			}
			else {//headphone is in
				mux_select(2, 2);
				unmute_headphone();				
			}
			
			EEPROM.write(0, 255 - selected_input);
			if(mute_state == LOW)
			{	//if amp is not mute we adjust led color. Green for aux and orange fo phono
				SoftPWMSetPercent(LED_pin, selected_input * 50);
			}
			Serial.println("switch input");
			switched_input_flag = HIGH;
			pb_flag=LOW;  
		}
		
	}

	if( mute_state == LOW){ //it reacts volume knob and headphone jack events only when not mute
		//digitalWrite(LED_pin, LOW);	//turn LED red
		//code for encoder increment_decrement *******************************
		if ((pb_rotation_flag==HIGH)){
			//multiplier for 0.5 dB change in volume is (15.3/256 + 1) which I'm going to approximate by (1/16 + 1)
			//dsp_vol_coef = 0ul;
			byte Headphone_dsp_vol_coef;			
			//read headphone volume******************************************************
			Wire.beginTransmission(PCM3070_address);
			Wire.write(0);	//go to page 1
			Wire.write(1);    		
			Wire.endTransmission(); 
			Wire.beginTransmission(PCM3070_address);
			Wire.write(22);	//go to Register 22 of page 1 for heaphone volume registers
			Wire.endTransmission();				
			Wire.requestFrom((uint8_t)PCM3070_address, (uint8_t)1);
			while(Wire.available()){ 
				Headphone_dsp_vol_coef = Wire.read();		
			}	

			//volume up*******************************************************************
			if (volume_change_direction == HIGH)
			{		
				Serial.println("Up");
				unsigned long temp_dsp_vol_coef = (dsp_vol_coef + (dsp_vol_coef>>2));	//increase volume by 2 dB
				char temp_amp_vol_coef = amp_vol_db + 2;
				if((temp_dsp_vol_coef < 0x788DB400ul)&&(temp_amp_vol_coef <= amp_vol_db_max)) 
				{
					dsp_vol_coef = temp_dsp_vol_coef;	
					amp_vol_db = temp_amp_vol_coef;
					write_volume_to_DSP(&dsp_vol_coef);
					write_volume_to_amp(amp_vol_db);
					Headphone_dsp_vol_coef -= 6;
					write_headphone_volume_to_DSP(Headphone_dsp_vol_coef);	//increase heaphone volume by 2 dB
				}
						
			}
			//volume down******************************************************************8
			else if (volume_change_direction == LOW)
			{
				Serial.println("Down");
				unsigned long temp_dsp_vol_coef = ((dsp_vol_coef/5)<<2);	//decrease volume by 2 dB
				char temp_amp_vol_coef = amp_vol_db - 2;
				if((temp_dsp_vol_coef > 0x00000E00ul)&&(temp_amp_vol_coef >= -79)) 
				{
					dsp_vol_coef = temp_dsp_vol_coef;
					amp_vol_db = temp_amp_vol_coef;
					write_volume_to_DSP(&dsp_vol_coef);
					write_volume_to_amp(amp_vol_db);					
					Headphone_dsp_vol_coef += 6;
					write_headphone_volume_to_DSP(Headphone_dsp_vol_coef);	//decrease heaphone volume by 2 dB		
				}
			}
			pb_rotation_flag=LOW;
		}
		
		//Code for headphone jack sense***************************************
		if ((headjackIn_flag==HIGH)&&(digitalRead(A1)==LOW)){	//headjack out
			mute_headphone();
			mux_select(1, 2);
			//unmute_subwoofer();
			unmute_amp();
			//Serial.println("headjack out");
			headjackIn_flag=LOW;
			
		}
		else if ((headjackIn_flag==HIGH)&&(digitalRead(A1)==HIGH)){	//headjack in
			//mute_subwoofer();
			mute_amp();
			mux_select(2, 2);
			//Serial.println("headjack in");
			headjackIn_flag=LOW;
			
			unmute_headphone();
		}				
	}
	else{	//when in mute state, keep the flags low.
		pb_rotation_flag = LOW;
		headjackIn_flag=LOW;
		//toggleLED();
	}		
	
	//code for subwoofer jack sense******************************************
	if ((subjack_flag==HIGH)&&(digitalRead(sub_jack)==LOW)){
		mux_select(2, 0);
		//Serial.println("subjack out");
		subjack_flag=LOW;
	}
	else if ((subjack_flag==HIGH)&&(digitalRead(sub_jack)==HIGH)){
		mux_select(1, 0);
		//Serial.println("subjack in");
		subjack_flag=LOW;
	}
	//VCC voltage monitoring*************************************************
	battVolts=getBandgap();	
	//if (battVolts != battVolts_old){Serial.println(battVolts); battVolts_old = battVolts;}	//check to volate fluctuations
	if(battVolts<300){	//In the event of a power loss mute everything
		//mute_amp();
		digitalWrite(TAS5708_PDN_pin, LOW);
		mute_amp();		
		mute_headphone();
		
		//Serial.println(battVolts);
		//asm volatile ("  jmp 0");	//restart program. 
	}
	
}

void switch_input(){
  unsigned int i = 0;
  unsigned int reg_off_reg_val;
	if(selected_input == 0){
		for(i=0; i<sizeof(aux_input_config)/2; i=i+1)
		{
			reg_off_reg_val = pgm_read_word_near(aux_input_config + i);
			Wire.beginTransmission(PCM3070_address);      
			Wire.write(reg_off_reg_val&0xff);
			Wire.write(reg_off_reg_val>>8);
			Wire.endTransmission(true);
		}	
	}
	else if (selected_input == 1){
		for(i=0; i<sizeof(phono_input_config)/2; i=i+1)
		{
			reg_off_reg_val = pgm_read_word_near(phono_input_config + i);
			Wire.beginTransmission(PCM3070_address);      
			Wire.write(reg_off_reg_val&0xff);
			Wire.write(reg_off_reg_val>>8);
			Wire.endTransmission(true);
		}	
	}

}

//FUNCTIONS

void PCM3070_Conf(){
 
  unsigned int i = 0;
  unsigned int reg_off_reg_val;
  //Serial.println(millis());
  for(i=0; i<sizeof(REG_Section_program)/2; i=i+1)
  {
    //reg_off of 254 indicates a delay of reg_val milliseconds. 
    // It doesn't actually get written to PCM. 
    // It can be used to indroduce a wait in the writing of configuration registers.
    // For example, after a reset or after the PLL has been enabled and needs time to stabalize
    reg_off_reg_val = pgm_read_word_near(REG_Section_program + i);
    if((reg_off_reg_val&0xff) == 254){  
       delay((unsigned long)(reg_off_reg_val>>8));
    }
    else if((reg_off_reg_val&0xff) == 255){  //255 is used to indicate when to start programming miniDSPs
      //program mini DSPs
      int j;
      for(j=0; j<(miniDSP_A_reg_values_COEFF_SIZE + miniDSP_A_reg_values_INST_SIZE) ; j++)
      {
        reg_off_reg_val = pgm_read_word_near(miniDSP_A_reg_values + j);
        Wire.beginTransmission(PCM3070_address);
        Wire.write(reg_off_reg_val&0xff);
        Wire.write(reg_off_reg_val>>8);    
        Wire.endTransmission();
      }
    
      for(j=0; j<(miniDSP_D_reg_values_COEFF_SIZE + miniDSP_D_reg_values_INST_SIZE) ; j++)
      {
        reg_off_reg_val = pgm_read_word_near(miniDSP_D_reg_values + j);
        Wire.beginTransmission(PCM3070_address);
        Wire.write(reg_off_reg_val&0xff);
        Wire.write(reg_off_reg_val>>8);    
        Wire.endTransmission();
      }      
    }  
    else
    {
      Wire.beginTransmission(PCM3070_address);      
      Wire.write(reg_off_reg_val&0xff);
      Wire.write(reg_off_reg_val>>8);
      Wire.endTransmission(true);
    }

  }

}



void TAS5708_Conf(){
//configuring TAS5708 
    //reseting the chip
    digitalWrite(TAS5708_reset_pin, LOW);
    delay(2);
    digitalWrite(TAS5708_reset_pin, HIGH);
    delay(20);

    //writing to configuration registers
	unsigned int i = 0;
	while(i < sizeof(TAS5708_conf))
	{
	  //do{
			unsigned int writeStopAddress = pgm_read_byte_near(TAS5708_conf + i)+i;
			i++;
			Wire.beginTransmission(0b00011011);
			
			for(i; i<=writeStopAddress; i++){	
				Wire.write(pgm_read_byte_near(TAS5708_conf + i));
				//Serial.println("1");
				//delay(1);
			}
			
			//Wire.write(TAS5708_conf, sizeof(TAS5708_conf));
			Wire.endTransmission();
			//	if(temp){ i = i -1 -writeStopAddress;Serial.println("error 432");}
		//}while(temp != 0);

	}    
    
}
// functions for volume increment-decrement
void volume_change_interrupt(){
  pb_rotation_flag=HIGH;
  volume_change_direction = digitalRead(encoder0PinB);
}
//functions for pushbotton interrupt
void pushbutton_interrupt(){
	pb_flag=HIGH;
	pb_flag_timeStamp=millis();
}

void subjack_interrupt(){
  subjack_flag=HIGH;
}
void headjackIn_interrupt(){
  headjackIn_flag=HIGH;
}
int getBandgap(void){// Returns actual value of Vcc (x 100)

 // For 168/328 boards
 const long InternalReferenceVoltage = 1056L;  // Adjust this value to your boards specific internal BG voltage x1000
	// REFS1 REFS0          --> 0 1, AVcc internal ref. -Selects AVcc external reference
	// MUX3 MUX2 MUX1 MUX0  --> 1110 1.1V (VBG)         -Selects channel 14, bandgap voltage, to measure
 ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0);
 delay(10);  // Let mux settle a little to get a more stable A/D conversion
	// Start a conversion  
 ADCSRA |= _BV( ADSC );
	// Wait for it to complete
 while( ( (ADCSRA & (1<<ADSC)) != 0 ) );
	// Scale the value
 int results = (((InternalReferenceVoltage * 1023L) / ADC) + 5L) / 10L; // calculates for straight line value 
 return results;

}
void mux_select(byte input_select, byte mux_index){
	Wire.beginTransmission(PCM3070_address);
	Wire.write(0x00);
	Wire.write(MUX_controls[mux_index].control_page );	//go to the right page
	Wire.endTransmission();

	Wire.beginTransmission(PCM3070_address);
	Wire.write(MUX_controls[mux_index].control_base+2);	//go the the right register. It seems PurePath studio uses control_base+2 for some reason instead of control_base+3 which is the LSB.
	Wire.write(input_select);	//select input 2 for MUX
	Wire.endTransmission();	
	
	//Switch buffers
	SwithchBuffers(MUX_controls[mux_index].control_page );
	delay(25);
	//After switching buffers, to keep them in-sync, we write the same thing in the other buffer 
	Wire.beginTransmission(PCM3070_address);
	Wire.write(0x00);
	Wire.write(MUX_controls[mux_index].control_page );	//go to the right page
	Wire.endTransmission();

	Wire.beginTransmission(PCM3070_address);
	Wire.write(MUX_controls[mux_index].control_base+2);	//go the the right register. It seems PurePath studio uses control_base+2 for some reason instead of control_base+3 which is the LSB.
	Wire.write(input_select);	//select input 2 for MUX
	Wire.endTransmission();	
	
}
bool write_volume_to_DSP(volatile unsigned long* vol){

	//0x16B54300
	if((vol[0] < 0x788DB400ul) && (vol[0] > 0x00000E00ul))
	{	
		//Serial.print("Volume to DSP: ");
		//Serial.println(dsp_vol_coef, HEX);	//print volume
	
		byte* dsp_vol_coef_pointer = (byte*)vol;
		
		Wire.beginTransmission(PCM3070_address);
		Wire.write(0x00);	//go to right page
		Wire.write(VOLUME_controls[0].control_page);   
		Wire.endTransmission(); 				
		Wire.beginTransmission(PCM3070_address);
		Wire.write(VOLUME_controls[0].control_base);	//go to the right register 
		Wire.write(dsp_vol_coef_pointer[3]); // = dsp_vol_coef*(1/16 + 1)
		Wire.write(dsp_vol_coef_pointer[2]); // = dsp_vol_coef*(1/16 + 1)
		Wire.write(dsp_vol_coef_pointer[1]); // = dsp_vol_coef*(1/16 + 1)
		Wire.write(dsp_vol_coef_pointer[0]);
		Wire.endTransmission(); 

		SwithchBuffers(VOLUME_controls[0].control_page );
		//After switching buffers, to keep them in-sync, we write the same thing in the other buffer 
		Wire.beginTransmission(PCM3070_address);
		Wire.write(0x00);	//go to right page
		Wire.write(VOLUME_controls[0].control_page);   
		Wire.endTransmission(); 			
		Wire.beginTransmission(PCM3070_address);
		Wire.write(VOLUME_controls[0].control_base);	//go to the right register 
		Wire.write(dsp_vol_coef_pointer[3]); // = dsp_vol_coef*(1/16 + 1)
		Wire.write(dsp_vol_coef_pointer[2]); // = dsp_vol_coef*(1/16 + 1)
		Wire.write(dsp_vol_coef_pointer[1]); // = dsp_vol_coef*(1/16 + 1)
		Wire.write(dsp_vol_coef_pointer[0]);
		Wire.endTransmission(); 		
		/*
		//Second volume control
		Wire.beginTransmission(PCM3070_address);
		Wire.write(0x00);	//go to right page
		Wire.write(VOLUME_controls[1].control_page);    		
		Wire.endTransmission(); 			
		Wire.beginTransmission(PCM3070_address);
		Wire.write(VOLUME_controls[1].control_base);	//go to the right register 
		Wire.write(dsp_vol_coef_pointer[3]); // = dsp_vol_coef*(1/16 + 1)
		Wire.write(dsp_vol_coef_pointer[2]); // = dsp_vol_coef*(1/16 + 1)
		Wire.write(dsp_vol_coef_pointer[1]); // = dsp_vol_coef*(1/16 + 1)
		Wire.write(dsp_vol_coef_pointer[0]);
		Wire.endTransmission(); 			
		
		SwithchBuffers(VOLUME_controls[1].control_page );
		Wire.beginTransmission(PCM3070_address);
		Wire.write(0x00);	//go to right page
		Wire.write(VOLUME_controls[1].control_page);    		
		Wire.endTransmission(); 			
		Wire.beginTransmission(PCM3070_address);
		Wire.write(VOLUME_controls[1].control_base);	//go to the right register 
		Wire.write(dsp_vol_coef_pointer[3]); // = dsp_vol_coef*(1/16 + 1)
		Wire.write(dsp_vol_coef_pointer[2]); // = dsp_vol_coef*(1/16 + 1)
		Wire.write(dsp_vol_coef_pointer[1]); // = dsp_vol_coef*(1/16 + 1)
		Wire.write(dsp_vol_coef_pointer[0]);
		Wire.endTransmission(); 			
		*/
		return true;
	}	
	else {	
		//Serial.print("Volume Out of range: ");
		//Serial.println(dsp_vol_coef, HEX);	//print volumereturn false;
		return false;
	}
}

void SwithchBuffers(byte page){
	//Switches either ADC buffer or DAC buffer, depending the page 
	if(page >= 44){//Switch DAC buffers		
		Wire.beginTransmission(PCM3070_address);
		Wire.write(0x00);	//go to right page
		Wire.write(0x2C);  	//44  		
		Wire.endTransmission();
		Wire.beginTransmission(PCM3070_address);
		Wire.write(0x01);	//go to right register
		Wire.write(0b00000101);    		
		Wire.endTransmission();
	}
	else if(page < 44){		//Switch ADC buffers
		Wire.beginTransmission(PCM3070_address);
		Wire.write(0x00);	//go to right page
		Wire.write(0x08);  	//44  		
		Wire.endTransmission();
		Wire.beginTransmission(PCM3070_address);
		Wire.write(0x01);	//go to right register
		Wire.write(0b00000101);    		
		Wire.endTransmission();
	}
}

bool write_headphone_volume_to_DSP(byte Headphone_dsp_vol_coef){

	
	if((Headphone_dsp_vol_coef < 0b01110010) && (Headphone_dsp_vol_coef > 0x00)){
		//Serial.print("Headphone Volume: ");
		//Serial.println(Headphone_dsp_vol_coef, HEX);
		Wire.beginTransmission(PCM3070_address);
		Wire.write(0);	//go to page 1
		Wire.write(1);    		
		Wire.endTransmission(); 
		Wire.beginTransmission(PCM3070_address);
		Wire.write(22);
		Wire.write(Headphone_dsp_vol_coef);
		Wire.write(Headphone_dsp_vol_coef);		
		Wire.endTransmission();	
		return true;
	}	
	else return false;
}
bool write_volume_to_amp(char volume_db){
	/*0x00 is 24dB	
	0x30 (48) is 0dB
	0xCE (206) is -79dB
	[0xCF to 0xFE] is reserved
	0xFF is mute*/
	if((volume_db>24)||(volume_db<-79))	return false;
	
	char volume = 48 - 2*volume_db; 
	if((volume>=0xCF)&&(volume<=0xFE)){
		return false;
	}

	/*
	At max volume, I'll have 19Vpp. 
	The driver can take 9Vpp @ 50hz before distorting, 13Vpp at 60hz, and 19Vpp at 70hz
	The DRC values should be adjusted in a way this Vpp at these frequencies does not exceed driver's dynamic range. 
	
	*/
	//adjust DRC
	//char temp =  - volume_db - 37;	louder
	
	//char temp =  - volume_db - 43;	quiter
	//int Threshold1 = int(temp);
	
	//int Threshold1 = (-1 * int(volume_db)*4/3) - 36;	//multiplying by 4/3 cause the increases in volume by completely canceled out by the compressor, which is like having a flat top (infinit compression ratio). This will cuase changes in input signal and changes to volume to have different effects on the level of bass. This doesn't make much sense so in the code below, I got rid of the 4/3 multiplier. 
	int compressionCurvePosition = -35; //Increasing this will cause the compression to kick in at higher levels. i.e. -40 compresses sooner than -35
	int Threshold1 = (-1 * int(volume_db)*1) + compressionCurvePosition;
	
	int Offset2 = Threshold1*3/4 - 3;	
	//Threshold1 = Threshold1 + 2;	//reduce low volume bass boost by 1.5 dB. from 6 dB to 4.5 dB
	
	if(Threshold1>=4)
	{
		//THreshold is raised when volume is lowered. We can't raise the threshold, below a certain volume. 
		Threshold1 = 4;
		Offset2 = 0;		
	}
	
	char Threshold1_char = char(-Threshold1);
	char Offset2_char = char(Offset2);
	
	//Threshold1_char = 0xF5;
	//Offset2_char = 0x00;

	uint8_t threshold1_regoffset = 5*4;
	uint8_t offset2_regoffset = 8*4;
	
	//noInterrupts();

	Wire.beginTransmission(PCM3070_address);
	Wire.write(0x00);	//go to right page
	Wire.write(DRC_controls[0].control_page);   
	Wire.endTransmission(); 				
	Wire.beginTransmission(PCM3070_address);
	Wire.write(DRC_controls[0].control_base + threshold1_regoffset);	//go to the right register 
	//Wire.write(char(int(28))); 
	Wire.write(Threshold1_char); 
	Wire.endTransmission(); 
	Wire.beginTransmission(PCM3070_address);
	Wire.write(DRC_controls[0].control_base + offset2_regoffset);	//go to the right register 
	//Wire.write(char(int(-24))); 
	Wire.write(Offset2_char); 
	Wire.endTransmission(); 	

	SwithchBuffers(DRC_controls[0].control_page );
	//After switching buffers, to keep them in-sync, we write the same thing in the other buffer 
	Wire.beginTransmission(PCM3070_address);
	Wire.write(0x00);	//go to right page
	Wire.write(DRC_controls[0].control_page);   
	Wire.endTransmission(); 				
	Wire.beginTransmission(PCM3070_address);
	Wire.write(DRC_controls[0].control_base + threshold1_regoffset);	//go to the right register 
	//Wire.write(char(int(28))); 
	Wire.write(Threshold1_char); 
	Wire.endTransmission(); 
	Wire.beginTransmission(PCM3070_address);
	Wire.write(DRC_controls[0].control_base + offset2_regoffset);	//go to the right register 
	//Wire.write(char(int(-24))); 
	Wire.write(Offset2_char); 
	Wire.endTransmission(); 	

	//interrupts();	


	//write amp gain
	Wire.beginTransmission(TAS5708_address);
	Wire.write(0x07);	//volume control register address
	Wire.write(volume);	//soft mute the AMP
	Wire.endTransmission();
	return true;

}
void mute_amp(){
	Wire.beginTransmission(TAS5708_address);
	Wire.write(0x06);
	Wire.write(0b00000011);	//soft mute the AMP
	Wire.endTransmission();
	
	mute_subwoofer();
}
void unmute_amp(){
	Wire.beginTransmission(TAS5708_address);
	Wire.write(0x06);
	Wire.write(0b00000000);	//soft unmute the AMP
	Wire.endTransmission(true);
	
	unmute_subwoofer();
}

void mute_headphone(){
	Wire.beginTransmission(PCM3070_address);
	Wire.write(0x00);
	Wire.write(0x01);	//go to page 1
	Wire.endTransmission();

	Wire.beginTransmission(PCM3070_address);
	Wire.write(16);
	Wire.write(0b01010000);	//reg[  1][ 16] = 0x00	; mute HPL driver, 0dB Gain
	Wire.write(0b01010000);	//reg[  1][ 17] = 0x00	; mute HPR driver, 0dB Gain
	Wire.endTransmission();
}
void unmute_headphone(){
	Wire.beginTransmission(PCM3070_address);
	Wire.write(0x00);
	Wire.write(0x01);	//go to page 1
	Wire.endTransmission();

	Wire.beginTransmission(PCM3070_address);
	Wire.write(16);
	Wire.write(0b00010000);	//reg[  1][ 16] = 0x00	; mute HPL driver, 0dB Gain
	Wire.write(0b00010000);	//reg[  1][ 17] = 0x00	; mute HPR driver, 0dB Gain
	Wire.endTransmission();
}

void unmute_subwoofer(){
	Wire.beginTransmission(PCM3070_address);
	Wire.write(0x00);
	Wire.write(0x01);	//go to page 1
	Wire.endTransmission();

	Wire.beginTransmission(PCM3070_address);
	Wire.write(14);
	Wire.write(0x08);	//reg[  1][ 16] = 0x00	; mute HPL driver, 0dB Gain
	Wire.write(0x08);	//reg[  1][ 17] = 0x00	; mute HPR driver, 0dB Gain
	Wire.endTransmission();
}
void mute_subwoofer(){
	Wire.beginTransmission(PCM3070_address);
	Wire.write(0x00);
	Wire.write(0x01);	//go to page 1
	Wire.endTransmission();

	Wire.beginTransmission(PCM3070_address);
	Wire.write(14);
	Wire.write(0x00);	//reg[  1][ 16] = 0x00	; mute HPL driver, 0dB Gain
	Wire.write(0x00);	//reg[  1][ 17] = 0x00	; mute HPR driver, 0dB Gain
	Wire.endTransmission();
}



