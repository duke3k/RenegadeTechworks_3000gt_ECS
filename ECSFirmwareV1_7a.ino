//****************************************************************************
//
//     Description of Software:    
//
//     Mitsubishi 3000gt/Stealth Custom Controller Program
//     for controlling stock OEM mitsubishi ECS Struts.
//     Hardware Platform:   ATAMega 328 MicroController
//
//     Copyright (C) 2014  Marcus Diaz, RenegadeTechWorks, LLC
//
//****************************************************************************
// Licensing:
//              Licensed under GNU GPL Version 2
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation under Version 2 of the License
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License along
//    with this program; if not, write to the Free Software Foundation, Inc.,
//    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//****************************************************************************
// VERSION HISTORY
// --------------------------------------------------------------------
// SW Ver  :  v1.0
// HWDesign:  ECM M01 PCB Rev1.1
// Date:      09/15/2012
// Comments:  Production version of code based on branch from original
//            prototype controller 
//            Diag Mode working - this version disables Select SW. 
//            check during Diag Mode Execution.'
//            This overcomes Voltage Drop.
//
//--------------------------------------------------------------------
// SW Ver  :  v1.1
// HWDesign:  ECM M01 PCB Rev1.1
// Date:      10/7/2012
// Comments:  Production version of code based on branch from original
//            prototype controller 
//            -v1.1 uses a Jumper accross Pins 6 <--> Pin3 on the
//             program header to indicate what type of selector switch
//             is being used as follows:
//                Jumper On : USE push Button ECS Switch
//                Jumper Off: Use Rotatry 4 Position Selector Switch
//            -v1.1 Has implemented the new re-written Diagonstic seq.
//                for using both LED's and OEM Tour/Sport Lights
//            -v1.1 also implements Diagnostic mode interrupt code
//            -v1.1 Implements a Dimming control option. To use at 
//                 start up put the system in Diagnostic Mode using
//                 using either the stock ECS switch by pressing and
//                 holding or Rotary switch to position 4. 
//*******************************************************************
// SW Ver  :  v1.1
// HWDesign:  ECM M01 PCB Rev1.1
// Date:      10/12/2012
// Comments:  Base  SVN production version of the code.
//            
//          Production version of code based on branch from original
//            prototype controller 
//            -v1.1 uses a Jumper accross Pins 6 <--> Pin3 on the
//             program header to indicate what type of selector switch
//             is being used as follows:
//                Jumper On : USE push Button ECS Switch
//                Jumper Off: Use Rotatry 4 Position Selector Switch
//            -v1.1 Has implemented the new re-written Diagonstic seq.
//                for using both LED's and OEM Tour/Sport Lights
//            -v1.1 also implements Diagnostic mode interrupt code
//            -v1.1 Implements a Dimming control option. To use at 
//                 start up put the system in Diagnostic Mode using
//                 using either the stock ECS switch by pressing and
//                 holding or Rotary switch to position 4. 
//
//--------------------------------------------------------------------
// SW Ver  :  v1.2
// HWDesign:  ECM M01 PCB Rev1.1
// Date:      10/13/2012
// Comments:  
//            added 2000 msec delay in diagonstic routine during display
//            of sucessfull strut test when both Tour & Sport lights
//            are turned on.            
//            
//--------------------------------------------------------------------
// SW Ver  :  v1.3
// HWDesign:  ECM M01 PCB Rev1.1
// Date:      10/23/2012
// Comments:  
//          - Corrected all code paths to correctly handle UNKNOWN Strut State
//          - Made Setup initialization routine more robust - all struts must aggree
//          - Also during setup Tour/Sport will go into error mode if there
//            was disagreement
//          - added displayStrut as derivative of original readstrut()
//          - doubled MAXTRIES to 10000 - helps with laggy struts 
//          - fixed bug in readSelector() in case of PUSHBUTTON - was 
//            missing " else return (strutState)" at end
//--------------------------------------------------------------------
// SW Ver  :  v1.5
// HWDesign:  ECM M01 PCB Rev1.1
// Date:      10/31/2012
// Comments:  
//          - Baselining this as the final production version for the first production
//            run. 
//          - Other than these comments there are no diffs between this and 
//          - the previous verson
//--------------------------------------------------------------------
// SW Ver  :  v1.6
// HWDesign:  ECM M01 PCB Rev1.1
// Date:      4/13/2014
// Comments:  
//          - Added saftey code to detect intermittent broken signal wire
//            and prevent motor driver burn out
//          - When broken wire detected - Both Tour Sport Lights will alternately flash
//          -added new diagnostic read status mode (only available from pushbutton)
//          -added new startup & error sequencing display routines for Sport/Tour Lights
//          -LEDs now Have Yellow Error Condition = Signal Wire Failure
//          -LEDs Red Error Condition = Motor command sequence failure.
//--------------------------------------------------------------------
// SW Ver  :  v1.7
// HWDesign:  ECM M01 PCB Rev1.1
// Date:      4/13/2014
// Comments:  
//          -Added EProm Read/Write Code during startup to retrieve/store LED 
//           brigtness defaults
//          -Increased MaxTries by 20% from 10000 to 12000
//--------------------------------------------------------------------
// SW Ver  :  v1.7
// HWDesign:  ECM M01 PCB Rev1.1
// Date:      4/17/2014
// Comments:  
//          -Minor display tweek for Diagnostic display of Sport / Tourlights
//           So that they just blink 3 times in groups of 3 regardless of which 
//           strut is being reported for a Motor Command Failure
//--------------------------------------------------------------------
// SW Ver  :  v1.7a
// HWDesign:  ECM M01 PCB Rev1.1
// Date:      4/17/2014
// Comments:  
//          -increased MaxTries to 15000 from 12000
//*****************************************************************************

#include <EEPROM.h>


#define strutFR 1
#define strutFL 2
#define strutRR 3
#define strutRL 4

#define HARD 	1
#define MEDIUM 	2
#define SOFT 	3
#define DIAG	4
#define READDIAG 5
#define UNKNOWN 7
#define NOCHANGE 10
#define STARTUP  11

#define D_RED	   1
#define D_GREEN	   2
#define D_BLUE	   3
#define D_TURQUOIS 4
#define D_PURPLE   5
#define D_YELLOW   6
#define D_STARTUP  7
#define D_HARD     8
#define D_MEDIUM   9
#define D_SOFT     10
#define D_WHITE    11
#define D_OFF      12 


#define RED100   0xFF0000 // Bright Red
#define GREEN100 0x00FF00 // Bright Green
#define BLUE100  0x0000FF // Bright Blue
#define RED001   0x010000 // Faint red
#define RED050   0x800000 // 1/2 red (0x80 = 128 out of 256)
#define RED025   0x300000 // 25% RED
#define GREEN050 0x008000 // half Green
#define GREEN025 0x003000 // 25% GREEN
#define BLUE050  0x000080 // half Blue
#define BLUE025  0x000030 // 25% BLUE
#define WHITE015 0x151515 // 15% White
#define WHITE010 0x101010 // 10% White
#define WHITE005 0x050505 // 05% White
#define WHITE001 0x020202 // 05% White
#define WHITE100 0xFFFFFF // 100% White

#define LEDOFF	       0x000000	
#define RED            0xFF0000 
#define ORANGE         0xFF5500
#define ORANGEYELLOW   0xFFA000
#define YELLOW         0xFFFF00
#define YELLOWGREEN    0xA0FF00
#define GREENYELLOW    0x50FF00
#define GREEN          0x00FF00
#define GREENCYAN      0x00FF4B
#define CYAN           0x00FFFF
#define CYANBLUE       0x00A0FF
#define BLUECYAN       0x005AFF  
#define BLUE           0x0000FF
#define BLUEVIOLET     0x4800FF
#define VIOLETBLUE     0x7500FF
#define MAGENTA        0xFF00FF
#define PINKDEEP       0xFF1493
#define PINKHOT        0xFF69B4
#define PINK           0xF3967A
 
#define VIOLET         0xEE82EE


#define NUMBER_LEDS 4

#define ERROR 1
#define NOERROR 0

#define SIGNAL_WIRE_FAILURE   2
#define MOTOR_COMMAND_FAILURE 1
#define NO_FAILURES 0

#define MAXTRIES 15000

#define MAXLIGHTLOOPS 20

#define ROTARY 1
#define PUSHBUTTON 2
#define SAMEMODE 1
#define NEXTMODE 2

#define LED_LEVEL1 1       // minimum brightness level
#define LED_LEVEL2 2
#define LED_LEVEL3 3
#define LED_LEVEL4 4       // Maximum brightness level

#define EPROM_LEDMODE 1   // address in EPROM for LEDMODE

int ledMode = LED_LEVEL2;

int selectorType = ROTARY;  // selectorType captures what kind of selector switch is hooked up to unit

// Previous State of Selector Switch
int currentSelectorState = STARTUP;

int strutState = HARD;
int strutStateFL;
int strutStateFR;
int strutStateRL;
int strutStateRR;

long ledHardValue ;
long ledMediumValue;
long ledSoftValue;
long ledRedValue;

long LED[NUMBER_LEDS];

int LED1 = 0;    // Front Left Strut LED
int LED2 = 1;    // Front Right Strut LED
int LED3 = 2;    // Rear Left Strut LED
int LED4 = 3;    // Rear Right Strut LED 

int errorFL = NO_FAILURES;
int errorFR = NO_FAILURES;
int errorRL = NO_FAILURES;
int errorRR = NO_FAILURES;

int tryCount = 0;

int lightLoopCounter = MAXLIGHTLOOPS;
int lightLoopCounterValue;

///////////////////////////////////////////////////////////////////////////////////////
//
//  Map Board I/O Pins to World
//
// /////////////////////////////////////////////////////////////////////////////////////
// Front Left Strut Pins
int strutFLswitch1 = 3; 
int strutFLswitch2 = 2;
int strutFLmotor   = 10;

// Front Right Strut Pins
int strutFRswitch1 = 5; 
int strutFRswitch2 = 4;
int strutFRmotor   = 11;

// Rear Left Strut Pins
int strutRLswitch1 = 7; 
int strutRLswitch2 = 6;
int strutRLmotor   = 14;

// Rear Right Strut Pins
int strutRRswitch1 = 9; 
int strutRRswitch2 = 8;
int strutRRmotor   = 15;

//Mode Selector Switch Pin
int ModeSelectSwitch = 17;

//Switch Type Selector Option Pin
int switchTypeSelect = 1;

//MotorEnable Pin
int MotorEnable = 16;

//
// LED Light Ports 
int SDI = 18;         //PIN Port for LED Array Data Signal
int CKI = 19;         //PIN Port for LED Array Clock Signal
                      // Note: See postLED() for statement

// Sport Light Port
int SportLight = 12; 

// Tour Light Port
int TourLight = 13;



 
///////////////////////////////////////////////////////////////////////////////////////
// setup initializes startup 
///////////////////////////////////////////////////////////////////////////////////////
void setup(){

        int selectorValue;
        int diagLoopNumber;

  	// initalize MotorEnable Pin for digital output and disable motors
	pinMode(MotorEnable, OUTPUT);
	digitalWrite(MotorEnable,LOW);
 
        // Initialize LED Array Pins 
        pinMode(SDI, OUTPUT);
        pinMode(CKI, OUTPUT);
        

        // Initialize the Sport & Tour Light OutPut Pins
        pinMode(SportLight, OUTPUT); 
        pinMode(TourLight, OUTPUT);
 
          // intialiaze all the motor out pins
        initializeMotorIOPin(strutFLmotor);
        initializeMotorIOPin(strutFRmotor);
        initializeMotorIOPin(strutRRmotor);
        initializeMotorIOPin(strutRLmotor);
    
        // Initialize all the strut input pins 
 	initializeStrutIOPin(strutFRswitch1);
 	initializeStrutIOPin(strutFRswitch2);
 	initializeStrutIOPin(strutFLswitch1);
 	initializeStrutIOPin(strutFLswitch2);
 	initializeStrutIOPin(strutRRswitch1);
 	initializeStrutIOPin(strutRRswitch2);
 	initializeStrutIOPin(strutRLswitch1);
 	initializeStrutIOPin(strutRLswitch2);
 

	// analyze Selector TYPE Input Pin To figure out if Rotary of Pushbutton is connected 
        // HIGH = Rotary   LOW = PUSHBUTTON
        pinMode(switchTypeSelect, INPUT);
        digitalWrite(switchTypeSelect,HIGH); 


        // Read the pin to see if it's HIGH or LOW      
        if(digitalRead(switchTypeSelect) == LOW){
             selectorType = PUSHBUTTON;
             pinMode(ModeSelectSwitch, INPUT);
             digitalWrite(ModeSelectSwitch, HIGH);
             lightLoopCounterValue = 900; // the lightloopcounter is timing dependent based on selector type
             diagLoopNumber = 450;
        }else{
             selectorType = ROTARY;
             pinMode(ModeSelectSwitch, INPUT);
             digitalWrite(ModeSelectSwitch, LOW);
             lightLoopCounterValue = 60;  // the lightloopcounter is timing dependent based on selector type
             diagLoopNumber = 40;
        } 
        lightLoopCounter = lightLoopCounterValue;

	//
	// Retrieve LED Brigtness level from EPROM - if not within range set it to 2nd level and write it back        //
        ledMode = eepromReadInt(EPROM_LEDMODE);	
	//
	// Make sure the value is with in range
	//
	if (ledMode != LED_LEVEL1 && ledMode != LED_LEVEL2 && ledMode != LED_LEVEL3 && ledMode != LED_LEVEL4){
	    // It's not within range - set to LEVEL two and write it back
	    ledMode = LED_LEVEL2;
            eepromWriteInt(EPROM_LEDMODE,ledMode);
	} 


	//
	// Blink the Sport / Tour Lights at startup as test to driver to show they are working.
	//
        blinkBothLights(1,3000,1,800);
	
  
	//
	//SET the LEDs to what the EPROM Value Says
	//
	switch(ledMode){
		case LED_LEVEL1:
	                 //SET the LEDs to Minimum Brightness Mode	
                         ledHardValue = rgbEncode(1,0,1);
                         ledMediumValue = rgbEncode(0,1,1);
                         ledSoftValue = rgbEncode(0,0,1);
                         ledRedValue = rgbEncode(1,0,0);

			break;
		case LED_LEVEL2:
	                 //SET the LEDs to 2nd to Minimum Brightness Mode	
                         ledHardValue = rgbEncode(10,0,10);
                         ledMediumValue = rgbEncode(0,10,5);
                         ledSoftValue = rgbEncode(0,0,10);
                         ledRedValue = rgbEncode(10,0,0);
			break;
		case LED_LEVEL3:
	                 //SET the LEDs to Medium Brightness Mode	
                         ledHardValue = rgbEncode(70,0,70);
                         ledMediumValue = rgbEncode(0,70,35);
                         ledSoftValue = rgbEncode(0,0,70);
                         ledRedValue = rgbEncode(70,0,0);
			break;
		case LED_LEVEL4:
	                 //SET the LEDs to Maximum Brightness Mode	
                         ledHardValue = rgbEncode(120,0,120);
                         ledMediumValue = rgbEncode(0,120,60);
                         ledSoftValue = rgbEncode(0,0,150);
                         ledRedValue = rgbEncode(120,0,0);
			break;
	}

        //  
        //See if user wants to set brightness level of LEDs
        //
        // Let them know it's time & wait 3 secs
        setLED(LED1, D_WHITE);
        setLED(LED2, D_WHITE);
        setLED(LED3, D_WHITE);
        setLED(LED4, D_WHITE);
        postLED();
        delay(2000);



	//read the selector PUSHBUTTON or Rotary and see if they have it in DIAGMODE
        if(readSelectorSwitch() == DIAG ){

            // ok..they want to adjust - put the leds into their medium value colors so they know
            strutState=DIAG;
            
	    //Default the LEDs to MEDIUM Mode	
            ledHardValue = rgbEncode(10,0,10);
            ledMediumValue = rgbEncode(0,10,5);
            ledSoftValue = rgbEncode(0,0,10);
            ledRedValue = rgbEncode(10,0,0);
            strutState = MEDIUM;
  
            setLED(LED1, D_HARD);
            setLED(LED2, D_MEDIUM);
            setLED(LED3, D_SOFT);
            setLED(LED4, D_RED);
            postLED();
                    
            // Loop for a while to let the adjust and then bail
	    for(int i = 1; i <diagLoopNumber; i++){
                  delay(20);
                   switch(readSelectorSwitch()){
                      case HARD:
	                 //SET the LEDs to Minimum Brightness Mode	
	                 ledMode = LED_LEVEL1;
                         ledHardValue = rgbEncode(1,0,1);
                         ledMediumValue = rgbEncode(0,1,1);
                         ledSoftValue = rgbEncode(0,0,1);
                         ledRedValue = rgbEncode(1,0,0);
  
                         setLED(LED1, D_HARD);
                         setLED(LED2, D_MEDIUM);
                         setLED(LED3, D_SOFT);
                         setLED(LED4, D_RED);
                         postLED();
                      break;

                      case MEDIUM:
	                 //SET the LEDs to 2nd to Minimum Brightness Mode	
	                 ledMode = LED_LEVEL2;
                         ledHardValue = rgbEncode(10,0,10);
                         ledMediumValue = rgbEncode(0,10,5);
                         ledSoftValue = rgbEncode(0,0,10);
                         ledRedValue = rgbEncode(10,0,0);
  
                         setLED(LED1, D_HARD);
                         setLED(LED2, D_MEDIUM);
                         setLED(LED3, D_SOFT);
                         setLED(LED4, D_RED);
                         postLED();
                      break;

                      case SOFT:
	                 //SET the LEDs to Medium Brightness Mode	
	                 ledMode = LED_LEVEL3;
                         ledHardValue = rgbEncode(70,0,70);
                         ledMediumValue = rgbEncode(0,70,35);
                         ledSoftValue = rgbEncode(0,0,70);
                         ledRedValue = rgbEncode(70,0,0);
  
                         setLED(LED1, D_HARD);
                         setLED(LED2, D_MEDIUM);
                         setLED(LED3, D_SOFT);
                         setLED(LED4, D_RED);
                         postLED();
                      break;
			   
		      case DIAG:
	                 //SET the LEDs to Maximum Brightness Mode	
	                 ledMode = LED_LEVEL4;
                         ledHardValue = rgbEncode(120,0,120);
                         ledMediumValue = rgbEncode(0,120,60);
                         ledSoftValue = rgbEncode(0,0,150);
                         ledRedValue = rgbEncode(120,0,0);
  
                         setLED(LED1, D_HARD);
                         setLED(LED2, D_MEDIUM);
                         setLED(LED3, D_SOFT);
                         setLED(LED4, D_RED);
                         postLED();
		      break;
                 }
	    }
	    //
	    // IF the new mode is not whats already in EProm Update EProm
	    //
            if(ledMode != eepromReadInt(EPROM_LEDMODE)) eepromWriteInt(EPROM_LEDMODE,ledMode);
        }
        
	// Let them know it's over
        setLED(LED1,D_WHITE);
        setLED(LED2,D_WHITE);
        setLED(LED3,D_WHITE);
        setLED(LED4,D_WHITE);
        delay(2000);
        
        // Set LEDS & Lights to current state of Struts 
        strutStateFL = displayStrut(strutFLswitch1,strutFLswitch2,LED1,ERROR);
        strutStateFR = displayStrut(strutFRswitch1,strutFRswitch2,LED2,ERROR);
        strutStateRL = displayStrut(strutRLswitch1,strutRLswitch2,LED3,ERROR);
        strutStateRR = displayStrut(strutRRswitch1,strutRRswitch2,LED4,ERROR); 
        
        
        // IF the state of all the struts match 
	// set strutState to whatever the Front Left Strut is reading	 
	if(strutStateFL == strutStateFR && 
	   strutStateFL == strutStateRL && 	
	   strutStateFL == strutStateRR && 	
	   strutStateFL != UNKNOWN ){
		
		  strutState=strutStateFL;
		  currentSelectorState = strutStateFL;
		  
                  // Set the TourSport Lights to indicate initial setting 
	          setTourSportLights();
        }else{
		 // not sure what state all struts are in. Arbitarily try and set to HARD
		 // Let user try and move to different state
		 strutState=HARD;
		 currentSelectorState = HARD;
                 setStrutMode( HARD, HARD, strutFLmotor, strutFLswitch1, strutFLswitch2,LED1, &errorFL);
                 setStrutMode( HARD, HARD, strutFRmotor, strutFRswitch1, strutFRswitch2,LED2, &errorFR);
                 setStrutMode( HARD, HARD, strutRLmotor, strutRLswitch1, strutRLswitch2,LED3, &errorRL);
                 setStrutMode( HARD, HARD, strutRRmotor, strutRRswitch1, strutRRswitch2,LED4, &errorRR);        

                 setTourSportLights();
	}
}


/////////////////////////////////////////////////////////////////////////////////////
//
//            main loop  
//
/////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  
   //
   // Make Sure the Strut Motors are OFF
   //
   digitalWrite(MotorEnable,LOW);
   digitalWrite(strutFRmotor, LOW);
   digitalWrite(strutFLmotor, LOW);
   digitalWrite(strutRRmotor, LOW);
   digitalWrite(strutRLmotor, LOW);

   //  SIGNAL_WIRE_FAILURE   
   //  MOTOR_COMMAND_FAILURE 
   //  NO_FAILURES 
 
   
   delay(20);
   //
   // Read the state of the Switch
   //   
     switch(readSelectorSwitch()){
	case(NOCHANGE):
                // Set LEDS & Lights to current state of Struts 
                if (errorFL == NO_FAILURES && currentSelectorState != displayStrut(strutFLswitch1,strutFLswitch2,LED1,ERROR)) {
			errorFL = SIGNAL_WIRE_FAILURE;
			setLED(LED1,D_YELLOW);
		}

                if (errorFR == NO_FAILURES && currentSelectorState != displayStrut(strutFRswitch1,strutFRswitch2,LED2,ERROR)){
			errorFR = SIGNAL_WIRE_FAILURE;
		       	setLED(LED2,D_YELLOW);
		}

                if (errorRL == NO_FAILURES && currentSelectorState != displayStrut(strutRLswitch1,strutRLswitch2,LED3,ERROR)){
			errorRL = SIGNAL_WIRE_FAILURE;
		       	setLED(LED3,D_YELLOW);
		}

                if (errorRR == NO_FAILURES && currentSelectorState != displayStrut(strutRRswitch1,strutRRswitch2,LED4,ERROR)){
			errorRR = SIGNAL_WIRE_FAILURE;
			setLED(LED4,D_YELLOW); 
		}

		setTourSportLights();
                break;

	case(HARD):

              	// Front Right Strut 
                setStrutMode( HARD, HARD, strutFLmotor, strutFLswitch1, strutFLswitch2,LED1, &errorFL);
                setStrutMode( HARD, HARD, strutFRmotor, strutFRswitch1, strutFRswitch2,LED2, &errorFR);
                setStrutMode( HARD, HARD, strutRLmotor, strutRLswitch1, strutRLswitch2,LED3, &errorRL);
                setStrutMode( HARD, HARD, strutRRmotor, strutRRswitch1, strutRRswitch2,LED4, &errorRR);        

		// Set the TourSport Lights to indicate if setting of all four was sucessfull or Error 
		setTourSportLights();
		break;

	case(MEDIUM):

                setStrutMode( MEDIUM, MEDIUM, strutFLmotor, strutFLswitch1, strutFLswitch2,LED1, &errorFL);
                setStrutMode( MEDIUM, MEDIUM, strutFRmotor, strutFRswitch1, strutFRswitch2,LED2, &errorFR);
                setStrutMode( MEDIUM, MEDIUM, strutRLmotor, strutRLswitch1, strutRLswitch2,LED3, &errorRL);
                setStrutMode( MEDIUM, MEDIUM, strutRRmotor, strutRRswitch1, strutRRswitch2,LED4, &errorRR);        

		// Set the TourSport Lights to indicate if setting of all four was sucessfull or Error 
		setTourSportLights();
		break;

	case(SOFT):
                
                setStrutMode( SOFT, SOFT, strutFLmotor, strutFLswitch1, strutFLswitch2,LED1, &errorFL);
                setStrutMode( SOFT, SOFT, strutFRmotor, strutFRswitch1, strutFRswitch2,LED2, &errorFR);
                setStrutMode( SOFT, SOFT, strutRLmotor, strutRLswitch1, strutRLswitch2,LED3, &errorRL);
                setStrutMode( SOFT, SOFT, strutRRmotor, strutRRswitch1, strutRRswitch2,LED4, &errorRR);        		 

		// Set the TourSport Lights to indicate if setting of all four was sucessfull or Error 
		setTourSportLights();
       	 	break;



	case(READDIAG):
                readDiagnostics();
       	 	break;

	case(DIAG):
                runDiagnostic();
       	 	break;

        case(UNKNOWN):
		setTourSportLights();
                break;

       }

}
///////////////////////////////////////////////////////////////////////////////////////
// Routine to Blink one of the lights
///////////////////////////////////////////////////////////////////////////////////////
void blinkLights(int lightaddress, int numberBlinks,int blinkDelay,int numberGroupBlinks,int groupDelay){
    
    setLights(lightaddress,LOW);
    
    for (int groupcount = numberGroupBlinks; groupcount > 0; groupcount--){
        
	for (int count = numberBlinks; count > 0; count--){
	   delay(blinkDelay);
	   setLights(lightaddress,HIGH);
	   delay(blinkDelay);
	   setLights(lightaddress,LOW);
        }
	delay(groupDelay);
    }
}
///////////////////////////////////////////////////////////////////////////////////////
// Routine to Blink one of the lights
///////////////////////////////////////////////////////////////////////////////////////
void blinkBothLights(int numberBlinks,int blinkDelay,int numberGroupBlinks,int groupDelay){
    
    // Turn off both Lights
    setLights(SportLight,LOW);
    setLights(TourLight,LOW);
    
    for (int groupcount = numberGroupBlinks; groupcount > 0; groupcount--){
        
	for (int count = numberBlinks; count > 0; count--){
	   delay(blinkDelay);
           setLights(SportLight,HIGH);
           setLights(TourLight,HIGH);
	   
	   delay(blinkDelay);
           setLights(SportLight,LOW);
           setLights(TourLight,LOW);
        }
	delay(groupDelay);
    }
}
///////////////////////////////////////////////////////////////////////////////////////
// Routine to Set TourSport Lights to CorrectMode
///////////////////////////////////////////////////////////////////////////////////////
void setTourSportLights(){

                lightLoopCounter--;
                if(lightLoopCounter <= 0){
                    lightLoopCounter = lightLoopCounterValue;
                    
  
  	            // if there was a problem start flashing shit.. to the do sport/tourlight error thing
		    if (errorFL == SIGNAL_WIRE_FAILURE || 
		        errorFR == SIGNAL_WIRE_FAILURE ||
		        errorRL == SIGNAL_WIRE_FAILURE ||
		        errorRR == SIGNAL_WIRE_FAILURE ){
  		     
                           for(int i=0;i<4;i++){
	                     setLights(SportLight,HIGH);
                             setLights(TourLight,LOW);
			     delay(250);
	                     setLights(SportLight,LOW);
                             setLights(TourLight,HIGH);
			     delay(250);
		           }
                    }
                          
                    if(errorFL == MOTOR_COMMAND_FAILURE || 
		       errorFR == MOTOR_COMMAND_FAILURE ||
		       errorRL == MOTOR_COMMAND_FAILURE ||
		       errorRR == MOTOR_COMMAND_FAILURE ) {
                            blinkBothLights(1,300,3,1000);
                           // blinkLights(SportLight, 3,800,1,1); 
                            delay(300); 
                    }   

		 }

                switch (strutState){
			   case(HARD):
	                     setLights(SportLight,HIGH);
                             setLights(TourLight,LOW);
			     break;

			   case(MEDIUM):
	                     setLights(SportLight,HIGH);
                             setLights(TourLight,HIGH);
			     break;

			   case(SOFT):
	                     setLights(SportLight,LOW);
                             setLights(TourLight,HIGH);
			     break;
			     
			   case(UNKNOWN):
	                     setLights(SportLight,LOW);
                             setLights(TourLight,LOW);
			     break;
		}
}
///////////////////////////////////////////////////////////////////////////////////////
// Routine to intialiaze IOPins for Strut Motors
///////////////////////////////////////////////////////////////////////////////////////
void initializeMotorIOPin(int pin){

	// Set pin used for strut motor as output 
	pinMode(pin, OUTPUT);

        // Make sure Motor is off
	digitalWrite(pin,LOW);
}

///////////////////////////////////////////////////////////////////////////////////////
// Routine to intialiaze IOPins for Strut Switches
///////////////////////////////////////////////////////////////////////////////////////
void initializeStrutIOPin(int pin){

	// Set pin used for strut motor as output 
	pinMode(pin, INPUT);

        // Pull Pin HIGH         
	digitalWrite(pin,HIGH);
}


///////////////////////////////////////////////////////////////////////////////////////
// Routine to set an LED  to a color by Address
///////////////////////////////////////////////////////////////////////////////////////
void setLED(int ledaddress, int color){
	//
        // rgbEncode(int red,int green,int blue)
	// red,green,blue =  0, 255
	//

	switch(color){
		case D_HARD:
			LED[ledaddress]=ledHardValue;
		        break;	
		case D_MEDIUM:
			LED[ledaddress]=ledMediumValue;
		        break;	
		case D_SOFT:
			LED[ledaddress]=ledSoftValue;
		        break;	
		case D_RED:
			LED[ledaddress]=ledRedValue;
		        break;	
		case D_GREEN:
			LED[ledaddress]=rgbEncode(0,10,0);
		        break;	
		case D_BLUE:
			LED[ledaddress]=rgbEncode(0,0,25);
		        break;	
		case D_TURQUOIS:
			LED[ledaddress]=rgbEncode(0,40,64);
		        break;	
		case D_PURPLE:
			LED[ledaddress]=rgbEncode(10,0,10);
		        break;	
		case D_YELLOW:
			LED[ledaddress]=rgbEncode(10,10,0);
		        break;
		case D_STARTUP:
			LED[ledaddress]=rgbEncode(2,2,2);
		        break;	
		case D_WHITE:
			LED[ledaddress]=rgbEncode(15,15,15);
		        break;	
		case D_OFF:
			LED[ledaddress]=rgbEncode(0,0,0);
		        break;	

	         
		}
         postLED(); //PUSH the current color definitions of ALL the LEDs out
}

///////////////////////////////////////////////////////////////////////////////////////
// Routine to Blink a  LED to a color by Address
///////////////////////////////////////////////////////////////////////////////////////
void blinkLED(int ledaddress, int color,int numberBlinks,int blinkdelay){
        
	switch(color){
		case D_HARD:
			for (int count = numberBlinks; count > 0; count--){
				delay(blinkdelay);
			        LED[ledaddress]=LEDOFF;
                                postLED(); //PUSH the current color definitions of ALL the LEDs out
				delay(blinkdelay);
			        LED[ledaddress]=rgbEncode(15,15,15);
                                postLED(); //PUSH the current color definitions of ALL the LEDs out
			}
		        break;	
		case D_MEDIUM:
			        
			for (int count = numberBlinks; count > 0; count--){
				delay(blinkdelay);
			        LED[ledaddress]=LEDOFF;
                                postLED(); //PUSH the current color definitions of ALL the LEDs out
				delay(blinkdelay);
			        LED[ledaddress]=rgbEncode(15,15,15);
                                postLED(); //PUSH the current color definitions of ALL the LEDs out
			}
		        break;	
		case D_SOFT:
		        for (int count = numberBlinks; count > 0; count--){
				delay(blinkdelay);
			        LED[ledaddress]=LEDOFF;
                                postLED(); //PUSH the current color definitions of ALL the LEDs out
				delay(blinkdelay);
			        LED[ledaddress]=rgbEncode(15,15,15);
                                postLED(); //PUSH the current color definitions of ALL the LEDs out
			}
		        break;	
		case D_RED:
			for (int count = numberBlinks; count > 0; count--){
				delay(blinkdelay);
			        LED[ledaddress]=LEDOFF;
                                postLED(); //PUSH the current color definitions of ALL the LEDs out
				delay(blinkdelay);
			        LED[ledaddress]=rgbEncode(15,15,15);
                                postLED(); //PUSH the current color definitions of ALL the LEDs out
			}
		        break;	
		case D_GREEN:
			for (int count = numberBlinks; count > 0; count--){
				delay(blinkdelay);
			        LED[ledaddress]=LEDOFF;
                                postLED(); //PUSH the current color definitions of ALL the LEDs out
				delay(blinkdelay);
			        LED[ledaddress]=rgbEncode(15,15,15);
                                postLED(); //PUSH the current color definitions of ALL the LEDs out
			}	
		        break;	
		case D_BLUE:
			for (int count = numberBlinks; count > 0; count--){
				delay(blinkdelay);
			        LED[ledaddress]=LEDOFF;
                                postLED(); //PUSH the current color definitions of ALL the LEDs out
				delay(blinkdelay);
			        LED[ledaddress]=rgbEncode(15,15,15);
                                postLED(); //PUSH the current color definitions of ALL the LEDs out
			}	
		        break;	
		case D_TURQUOIS:
			for (int count = numberBlinks; count > 0; count--){
				delay(blinkdelay);
			        LED[ledaddress]=LEDOFF;
                                postLED(); //PUSH the current color definitions of ALL the LEDs out
				delay(blinkdelay);
			        LED[ledaddress]=rgbEncode(15,15,15);
                                postLED(); //PUSH the current color definitions of ALL the LEDs out
			}	
		        break;	
		case D_PURPLE:
			for (int count = numberBlinks; count > 0; count--){
				delay(blinkdelay);
			        LED[ledaddress]=LEDOFF;
                                postLED(); //PUSH the current color definitions of ALL the LEDs out
				delay(blinkdelay);
			        LED[ledaddress]=rgbEncode(15,15,15);
                                postLED(); //PUSH the current color definitions of ALL the LEDs out
			}	
		        break;	
		case D_YELLOW:
			for (int count = numberBlinks; count > 0; count--){
				delay(blinkdelay);
			        LED[ledaddress]=LEDOFF;
                                postLED(); //PUSH the current color definitions of ALL the LEDs out
				delay(blinkdelay);
			        LED[ledaddress]=rgbEncode(15,15,15);;
                                postLED(); //PUSH the current color definitions of ALL the LEDs out
			}	
		        break;
		case D_STARTUP:
			for (int count = numberBlinks; count > 0; count--){
				delay(blinkdelay);
			        LED[ledaddress]=LEDOFF;
                                postLED(); //PUSH the current color definitions of ALL the LEDs out
				delay(blinkdelay);
			        LED[ledaddress]=rgbEncode(15,15,15);
                                postLED(); //PUSH the current color definitions of ALL the LEDs out
			}	
		        break;	
		case D_WHITE:
			for (int count = numberBlinks; count > 0; count--){
				delay(blinkdelay);
			        LED[ledaddress]=LEDOFF;
                                postLED(); //PUSH the current color definitions of ALL the LEDs out
				delay(blinkdelay);
			        LED[ledaddress]=rgbEncode(15,15,15);
                                postLED(); //PUSH the current color definitions of ALL the LEDs out
			}	
		        break;	
		}
}

///////////////////////////////////////////////////////////////////////////////////////
// Routine to turn an LED off
//////////////////////////////////////////////////////////////////////////////////////
void offLED(int ledaddress){
        
        LED[ledaddress]=LEDOFF;
        postLED(); //PUSH the current color definitions of ALL the LEDs out
}
///////////////////////////////////////////////////////////////////////////////////////
// Routine to set either the Tour Or Sport light to an HIGH/ON or LOW/OFF state
///////////////////////////////////////////////////////////////////////////////////////
void setLights(int light, int state){
  digitalWrite(light,state);
        
}

/////////////////////////////////////////////////////////////////////////////////////
// Reference 5v = 1023
//
// Hard     0.0v     / xxx
//    Cutoff  0.415  / 85
// Medium   0.83v    / xxx
//    Cutoff   1.245   / 255
// Soft     1.66v    / xxx
//    Cutoff   2.075   / 424
// Diag     2.5      / xxx
//    Cutoff   2.915   / 596
/////////////////////////////////////////////////////////////////////////////////////
int readSelectorSwitch(){
	int voltage;
	int voltage2;
	float voltdiff;

	// Based on the selector switch type do the appropriate read
	if(selectorType == ROTARY){
		
	   voltage = analogRead(ModeSelectSwitch)+1; // read the voltage	
	   delay(500);                               // debounce for 500 msec
	   voltage2 = analogRead(ModeSelectSwitch)+1;// read the voltage again
	   voltdiff = abs(1.0-voltage/voltage2);     // Calculate the percentage difference 

	   //
	   // as long as voltage difference > 10% keep reading
	   //
	   while(voltdiff > .1 ){
	      voltage = analogRead(ModeSelectSwitch)+1; // read the voltage	
	      delay(500);                             // debounce for 500 msec
	      voltage2 = analogRead(ModeSelectSwitch)+1;// read the voltage again
	      voltdiff = abs(1.0-voltage/voltage2);   // Calculate the percentage difference 
	   }

           //
	   // based on the converted voltage value and the range return the value
           //
	   if(voltage < 85){                          
             // Selector is in HARD mode 
             if (currentSelectorState == HARD) return (NOCHANGE) ;
	     else {
               if ( currentSelectorState != STARTUP ) currentSelectorState =HARD;
	       strutState = HARD;
               return(HARD);
	     }
	 
	   }else if (voltage >= 85 && voltage < 255){ 
	     // Selector is in MEDIUM mode 
             if (currentSelectorState == MEDIUM) return (NOCHANGE) ;
	     else {
               if ( currentSelectorState != STARTUP ) currentSelectorState = MEDIUM;
               strutState = MEDIUM;
	       return(MEDIUM);
	     }

	   }else if (voltage >= 255 && voltage < 424){
             // Selector is in SOFT mode 
             if (currentSelectorState == SOFT) return (NOCHANGE) ;
	     else {
               if ( currentSelectorState != STARTUP ) currentSelectorState = SOFT;
               strutState = SOFT;
	       return(SOFT);
	     }

	   }else if(voltage >= 424 && voltage <596){
             // Selector is in DIAG mode 
             if (currentSelectorState == DIAG) return (NOCHANGE) ;
	     else {
               if ( currentSelectorState != STARTUP) currentSelectorState = DIAG;
               strutState = DIAG;
	       return(DIAG);
	     }

	   }else{
              strutState=UNKNOWN;
	      return (UNKNOWN);

	   }

	}else if (selectorType == PUSHBUTTON){
            // if they are pressing switch i.e. LOW then decide what's next
            if(digitalRead(ModeSelectSwitch) == LOW){
                  delay(1000);
		  //
		  // If they are no longer pressing switch then shift the struts to next state
		  //
                  if(digitalRead(ModeSelectSwitch) == HIGH){
                        switch(strutState){
                           case HARD:
                             currentSelectorState = MEDIUM;
                             strutState = MEDIUM;
                             return(MEDIUM);
                           break;

                           case MEDIUM:
                             currentSelectorState = SOFT;
		             strutState = SOFT; 
                             return(SOFT);
                           break;

                           case SOFT:
                             currentSelectorState = HARD;
		             strutState = HARD; 
                             return(HARD);
                           break;
			   
			   case READDIAG:
			   case DIAG:
			   case UNKNOWN:
                             currentSelectorState = DIAG;
		             strutState = HARD; 
                             return(HARD);
                           break;
                   }
	        }else{
		     //
		     // They are still holding button down wait 3 secs
                     delay(3000);

		     //If they are still holding the switch after 3secs - see if they want DIAGNOSTIC or READ DIAG mode
                     if(digitalRead(ModeSelectSwitch) == LOW){ 
                         blinkBothLights(3,150,2,1000); // Blink both Lights to let them know they are at this mode
		         delay (3000);

                         if(digitalRead(ModeSelectSwitch) == LOW){ 
                              //
                              // Blink both Lights to let them know they are at this mode
			      // They should let go now after seeing blinks if they want to run diag mode
			      // 
                               blinkBothLights(3,150,2,1000); // Blink both Lights to let them know they are at this mode
		               delay (3000);	    

		             //If they let go before 3secs - then they want DIAGNOSTIC 
                             if(digitalRead(ModeSelectSwitch) == HIGH){ 
 			        // Ok they let go they want to Run Diagnostic Mode
                                strutState = DIAG; 
                                return(strutState);
			     } else return (strutState);
				 
			 }else {
			    // Ok they let go they want to Read the current Diagnostic Codes
                            strutState = READDIAG;
                            return(strutState);
			 }

		     } else return (strutState);   //Nope they dont' want anything - just return current State.
               }
           
	   }else if (currentSelectorState == DIAG && strutState == HARD){
	       // if here then this is 2nd pass thru just after comming out of DIAG or READDIAG MODE
               // Switch was not being pressed and last state was DIAG mode Now RETURN HARD to MAIN loop one more time
               currentSelectorState = HARD;
               return(HARD);
	   } else return(NOCHANGE);  // Switch is not being pressed - return NOCHANGE state
    }
}

/////////////////////////////////////////////////////////////////////////////////////
// Reference 5v = 1023
//
// Hard     0.0v     / xxx
//    Cutoff  0.415  / 85
// Medium   0.83v    / xxx
//    Cutoff   1.245   / 255
// Soft     1.66v    / xxx
//    Cutoff   2.075   / 424
// Diag     2.5      / xxx
//    Cutoff   2.915   / 596
/////////////////////////////////////////////////////////////////////////////////////
int readDiagSelectorSwitch(){
	int voltage;
	int voltage2;
	float voltdiff;

	// Based on the selector switch type do the appropriate read
	if(selectorType == ROTARY){
		
	   voltage = analogRead(ModeSelectSwitch)+1; // read the voltage	
	   delay(500);                               // debounce for 500 msec
	   voltage2 = analogRead(ModeSelectSwitch)+1;// read the voltage again
	   voltdiff = abs(1.0-voltage/voltage2);     // Calculate the percentage difference 

	   //
	   // as long as voltage difference > 10% keep reading
	   //
	   while(voltdiff > .1 ){
	      voltage = analogRead(ModeSelectSwitch)+1; // read the voltage	
	      delay(500);                             // debounce for 500 msec
	      voltage2 = analogRead(ModeSelectSwitch)+1;// read the voltage again
	      voltdiff = abs(1.0-voltage/voltage2);   // Calculate the percentage difference 
	   }
           currentSelectorState = DIAG;

           //
	   // based on the converted voltage value and the range return the value
           //
	   if(voltage < 85) return (HARD) ;
	   else if (voltage >= 85 && voltage < 255)  return (MEDIUM) ;
	   else if (voltage >= 255 && voltage < 424) return (SOFT);
	   else if(voltage >= 424 && voltage <596) return (NOCHANGE);
	   else return (UNKNOWN);


	}else if (selectorType == PUSHBUTTON){
            // if they are pressing switch i.e. LOW then decide what's next
            if(digitalRead(ModeSelectSwitch) == LOW){
                 blinkBothLights(3,150,2,1000); // Blink both Lights to let them know they are at this mode
		 delay (2000);
		  //
		  // they are pressing switch then shift the struts out of DIAG mode to HARD 
		  //
                  currentSelectorState = DIAG;
		  strutState = HARD; 
                  return(HARD);
            }else return (NOCHANGE);
         }
}

///////////////////////////////////////////////////////////////////////////////////////
//
//
// Routine to Read actual status of strut , set LED color apprropriately and return value
//
//
///////////////////////////////////////////////////////////////////////////////////////
int readstrut(int strutS1,int strutS2){
  int switch1;
  int switch2;


  switch2 = !digitalRead(strutS1);
  switch1 = !digitalRead(strutS2);
  
  if (switch1 == LOW && switch2 == HIGH){
    // Strut is in HARD mode
    return(HARD);
    
  }else if (switch1 == HIGH &&  switch2 == HIGH){
    // Strut is in MEDIUM mode  
    return(MEDIUM);
    
  }else if (switch1 == HIGH && switch2 == LOW){
    // Strut is in SOFT Mode
    return(SOFT);
  }else {
    return(UNKNOWN);
  }
}

///////////////////////////////////////////////////////////////////////////////////////
//
//
// Routine to Read actual status of strut , set LED color apprropriately and return value
//
//
///////////////////////////////////////////////////////////////////////////////////////
int displayStrut(int strutS1,int strutS2,int strutLED,int displayMode){
  int switch1;
  int switch2;


  switch2 = !digitalRead(strutS1);
  switch1 = !digitalRead(strutS2);
  
  if (switch1 == LOW && switch2 == HIGH){
    // Strut is in HARD mode
    setLED(strutLED, D_HARD);
    return(HARD);
    
  }else if (switch1 == HIGH &&  switch2 == HIGH){
    // If Strut is in MEDIUM mode  
    setLED(strutLED, D_MEDIUM);
    return(MEDIUM);
    
  }else if (switch1 == HIGH && switch2 == LOW){
    // Strut is in SOFT Mode
    setLED(strutLED, D_SOFT);
    return(SOFT);
  }else {
    if(displayMode == ERROR) setLED(strutLED, D_RED);
    return(UNKNOWN);
  }
}
///////////////////////////////////////////////////////////////////////////////////////
//
//
// Routine to Set the desired strut to a particular mode        
//
/////////////////////////////////////////////////////////////////////////////////////
int setStrutMode( int selectorDesiredMode, int strutDesiredMode, int strutMotor, int strutSwitch1, int strutSwitch2,int displayLED, int *strutErrorStatus){
  int tryCount;
  tryCount = 0;

   //  SIGNAL_WIRE_FAILURE   
   //  MOTOR_COMMAND_FAILURE 
   //  NO_FAILURES 

                //
		// IF any previous failures - display status & kick out 
		//
                if (*strutErrorStatus == MOTOR_COMMAND_FAILURE ){
                        setLED(displayLED,D_RED);
			return(ERROR);
		}else if (*strutErrorStatus == SIGNAL_WIRE_FAILURE){
                        setLED(displayLED,D_YELLOW);
			return(ERROR);
		}
                //
		// ok so no previous problems with this strut - get readay to set to new mode
                // 
		
                //Enable motor circuits  
                digitalWrite(MotorEnable,HIGH);
                
                //
	   	// Turn on motor as long as strut is not in desired mode but only try the loop for MAXTRIES number of times
                //
     	        while( displayStrut(strutSwitch1, strutSwitch2,displayLED,NOERROR) != strutDesiredMode &&
		       tryCount < MAXTRIES ){
			
        		digitalWrite(strutMotor, HIGH);
  		        tryCount++;
    		}
		//
		// Turn Everything Off
		//
    		digitalWrite(strutMotor, LOW);
                digitalWrite(MotorEnable,LOW);
    
     	        // if  MaxTries not exceeded strut is now in desired mode
  		if(tryCount < MAXTRIES ){

     	            // Strut is now in desired mode
                     *strutErrorStatus = NO_FAILURES;
                     return (NOERROR);

                }else {
		    //
		    // Maxtries was exceeded - display RED for 2 secs turn off for 1 sec & try again.
		    //
                    setLED(displayLED,D_RED);
                    delay(2000);
                    setLED(displayLED,D_OFF);
                    delay(1000);
		    //
		    // Try One More Time
		    // 
		    tryCount = 0;

                    //Enable motor circuits  
                    digitalWrite(MotorEnable,HIGH);

     	            while( displayStrut(strutSwitch1, strutSwitch2,displayLED,NOERROR) != strutDesiredMode && 
			  tryCount < MAXTRIES ) {

        		  digitalWrite(strutMotor, HIGH);
  		          tryCount++;
    		    }
		    //
		    // Turn Everything Off
		    // 
    		    digitalWrite(strutMotor, LOW);
                    digitalWrite(MotorEnable,LOW);
    
  		    if(tryCount < MAXTRIES){
     	                // Strut Managed to get into desired mode on 2nd try
                        *strutErrorStatus = NO_FAILURES;
                         return (NOERROR);
  		    }else{
		       //
		       // Maxtries was exceeded - Flag this strut as bad.
		       //
                        setLED(displayLED,D_RED);
                        *strutErrorStatus = MOTOR_COMMAND_FAILURE;
                        return (ERROR);
  		    }
		}
		 
}

//*******************************************************************************************
//
// Diagnostic routine
//
//
//*******************************************************************************************
void runDiagnostic(){
  
             while(readDiagSelectorSwitch()== NOCHANGE){ 
                   errorFR=0;
                   errorFL=0;
                   errorRR=0;
                   errorRL=0;
                   
                   // Turn Off All LEDS
                   offLED(LED1);
                   offLED(LED2);
                   offLED(LED3);
                   offLED(LED4);
                   
                   // Turn off Lights
	           setLights(SportLight,LOW);
                   setLights(TourLight,LOW);
                   delay(1000);
  
                   // See if abort desired
                   if(readDiagSelectorSwitch()!= NOCHANGE) return; 

                   //////////////////////////////////////////////////////////////////////////////////////
                   //                    TEST FRONT LEFT STRUT
                   ///////////////////////////////////////////////////////////////////////////////////////
		   
		   
                   //SET LED 1 to White to indicate testing Front Left Strut 
                   setLED(LED1,D_WHITE);
		   
		   // Blink Sport Light 1 time as group 4 times To indicate FL strut
                   blinkLights(SportLight, 1,300,4,1000);  

		   // See if abort desired
                   if(readDiagSelectorSwitch()!= NOCHANGE) return; 

                   // Do the test for this strut as many times as desired
	    	   for(int i =1; i<= 1; i++){

                      //Attempt to set  Front Left Strut HARD MODE
                      setStrutMode( DIAG, HARD, strutFLmotor, strutFLswitch1, strutFLswitch2,LED1, &errorFL);
                      delay(1000);
                   
		      // See if abort desired
		      if(readDiagSelectorSwitch()!= NOCHANGE) return; 
                                      

                      //Attempt to set  Front Left Strut Medium MODE
                      setStrutMode( DIAG, MEDIUM, strutFLmotor, strutFLswitch1, strutFLswitch2,LED1, &errorFL);
                      delay(1000);
		      
		      // See if abort desired
		      if(readDiagSelectorSwitch()!= NOCHANGE) return; 

                                      
                      //Attempt to set  Front Left Strut Soft MODE
                      setStrutMode( DIAG, SOFT, strutFLmotor, strutFLswitch1, strutFLswitch2,LED1, &errorFL);
                      delay(1000);

		      // See if abort desired
		      if(readDiagSelectorSwitch()!= NOCHANGE) return; 
		      
		   }

		   // Display Final test results for this Strut
		   if (errorFL == 0){
                       setLED(LED1,D_GREEN);
		       setLights(TourLight,HIGH);
		       setLights(SportLight,HIGH);
                       delay(2500);
		   }else{
                      blinkBothLights(3,300,3,1000);
                      setLED(LED1,D_RED);
		   }

		   // See if abort desired
		   if(readDiagSelectorSwitch()!= NOCHANGE) return; 
		   
                   delay(2000);

                   
                   //////////////////////////////////////////////////////////////////////////////////////
                   //                    TEST FRONT Right STRUT
                   ///////////////////////////////////////////////////////////////////////////////////////
		   // Turn off Lights
	           setLights(SportLight,LOW);
                   setLights(TourLight,LOW);
		   
                   //SET LED 2 to White to indicate testing Front Right Strut 
                   setLED(LED2,D_WHITE);
		   
		   // Blink Sport Light 2 time as group 3 times To indicate FR strut  
                   blinkLights(SportLight, 2,300,4,1000);  

		   // See if abort desired
                   if(readDiagSelectorSwitch()!= NOCHANGE) return; 

                   // Do the test for this strut as many times as desired
	    	   for(int i =1; i<= 1; i++){

                      //Attempt to set  Front Right Strut HARD MODE
                      setStrutMode( DIAG, HARD, strutFRmotor, strutFRswitch1, strutFRswitch2,LED2, &errorFR);
                      delay(1000);
                   
		      // See if abort desired
		      if(readDiagSelectorSwitch()!= NOCHANGE) return; 
		      

                      //Attempt to set  Front Right Strut Medium MODE
                      setStrutMode( DIAG, MEDIUM, strutFRmotor, strutFRswitch1, strutFRswitch2,LED2, &errorFR);
                      delay(1000);
		      
		      // See if abort desired
		      if(readDiagSelectorSwitch()!= NOCHANGE) return; 

                                      
                      //Attempt to set  Front Left Strut Soft MODE
                      setStrutMode( DIAG, SOFT, strutFRmotor, strutFRswitch1, strutFRswitch2,LED2, &errorFR);
                      delay(1000);

		      // See if abort desired
		      if(readDiagSelectorSwitch()!= NOCHANGE) return; 
		      
		      // Turn off Lights
		      setLights(SportLight,LOW);
                      setLights(TourLight,LOW);
		   }

		   // Display Final test results for this Strut
		   if (errorFR == 0){
                       setLED(LED2,D_GREEN);
		       setLights(TourLight,HIGH);
		       setLights(SportLight,HIGH);
                       delay(2500);
		   }else{
                      blinkBothLights(3,300,3,1000);
                      setLED(LED2,D_RED);
		   }

		   // See if abort desired
		   if(readDiagSelectorSwitch()!= NOCHANGE) return; 
		   
                   delay(2000);
  


                   //////////////////////////////////////////////////////////////////////////////////////
                   //                    TEST REAR LEFT STRUT
                   ///////////////////////////////////////////////////////////////////////////////////////
		   // Turn off Lights
	           setLights(SportLight,LOW);
                   setLights(TourLight,LOW);
		   
                   //SET LED 3 to White to indicate testing Front Left Strut 
                   setLED(LED3,D_WHITE);
		   
		   // Blink Sport Light 3 time as group 3 times To indicate RL strut 
                   blinkLights(SportLight, 3,300,4,1000); 

		   // See if abort desired
                   if(readDiagSelectorSwitch()!= NOCHANGE) return; 

                   // Do the test for this strut as many times as desired
	    	   for(int i =1; i<= 1; i++){

                      //Attempt to set  Rear Left Strut HARD MODE
                      setStrutMode( DIAG, HARD, strutRLmotor, strutRLswitch1, strutRLswitch2,LED3, &errorRL);
                      delay(1000);
                   
		      // See if abort desired
		      if(readDiagSelectorSwitch()!= NOCHANGE) return; 
		      
                      //Attempt to set  Rear Left Strut Medium MODE
                      setStrutMode( DIAG, MEDIUM, strutRLmotor, strutRLswitch1, strutRLswitch2,LED3, &errorRL);
                      delay(1000);
		      
		      // See if abort desired
		      if(readDiagSelectorSwitch()!= NOCHANGE) return; 

                                      
                      //Attempt to set  Rear Left Strut Soft MODE
                      setStrutMode( DIAG, SOFT, strutRLmotor, strutRLswitch1, strutRLswitch2,LED3, &errorRL);
                      delay(1000);

		      // See if abort desired
		      if(readDiagSelectorSwitch()!= NOCHANGE) return; 
		      

		      // Turn off Lights
		      setLights(SportLight,LOW);
                      setLights(TourLight,LOW);
		   }

		   // Display Final test results for this Strut
		   if (errorRL == 0){
                       setLED(LED3,D_GREEN);
		       setLights(TourLight,HIGH);
		       setLights(SportLight,HIGH);
                       delay(2500);
		   }else{
                      blinkBothLights(3,300,3,1000);
                      setLED(LED3,D_RED);

		   }

		   // See if abort desired
		   if(readDiagSelectorSwitch()!= NOCHANGE) return; 
		   
                   delay(2000);
  

                   //////////////////////////////////////////////////////////////////////////////////////
                   //                    TEST REAR RIGHT STRUT
                   ///////////////////////////////////////////////////////////////////////////////////////
		   // Turn off Lights
	           setLights(SportLight,LOW);
                   setLights(TourLight,LOW);
		   
                   //SET LED 1 to White to indicate testing Rear Right Strut 
                   setLED(LED4,D_WHITE);
		   
		   // Blink Sport Light 4 time as group 3 times To indicate RR strut 
                   blinkLights(SportLight, 4,300,4,1000);  

		   // See if abort desired
                   if(readDiagSelectorSwitch()!= NOCHANGE) return; 

                   // Do the test for this strut as many times as desired
	    	   for(int i =1; i<= 1; i++){

                      //Attempt to set  Rear Right Strut HARD MODE
                      setStrutMode( DIAG, HARD, strutRRmotor, strutRRswitch1, strutRRswitch2,LED4, &errorRR);        
                      delay(1000);
                   
		      // See if abort desired
		      if(readDiagSelectorSwitch()!= NOCHANGE) return; 
		      

                      //Attempt to set  Rear Right Strut Medium MODE
                      setStrutMode( DIAG, MEDIUM, strutRRmotor, strutRRswitch1, strutRRswitch2,LED4, &errorRR);        
                      delay(1000);
		      
		      // See if abort desired
		      if(readDiagSelectorSwitch()!= NOCHANGE) return; 

                                      
                      //Attempt to set  Rear Right Strut Soft MODE
                      setStrutMode( DIAG, SOFT, strutRRmotor, strutRRswitch1, strutRRswitch2,LED4, &errorRR);        
                      delay(1000);

		      // See if abort desired
		      if(readDiagSelectorSwitch()!= NOCHANGE) return; 
		      

		      // Turn off Lights
		      setLights(SportLight,LOW);
                      setLights(TourLight,LOW);
		   }

		   // Display Final test results for this Strut
		   if (errorRR == 0){
                       setLED(LED4,D_GREEN);
		       setLights(TourLight,HIGH);
		       setLights(SportLight,HIGH);
                       delay(2500);
		   }else{
                      blinkBothLights(3,300,3,1000);
                      setLED(LED4,D_RED);
		   }

		   
		   
                   delay(2000);
          }
          return; 

}
//*******************************************************************************************
//
//Read Diagnostic routine
//This routine displays the current error status of all the struts based on current statuses incurred
//this is mainly for Cars that have Sports Tour Lights Only
//
//*******************************************************************************************
void readDiagnostics(){
                   
                   //  Possible Error Codes for errorFL, errorFR, errorRL, errorRR  =
                   //	
                   //         SIGNAL_WIRE_FAILURE   
                   //         MOTOR_COMMAND_FAILURE 
                   //         NO_FAILURES 
                   //

         while( readDiagSelectorSwitch()== NOCHANGE){  


                   // Turn Off All LEDS
                   offLED(LED1);
                   offLED(LED2);
                   offLED(LED3);
                   offLED(LED4);
                   
                   // Turn off Lights
	           setLights(SportLight,LOW);
                   setLights(TourLight,LOW);
                   delay(1000);
  		   
                   // See if abort desired
                   if(readDiagSelectorSwitch()!= NOCHANGE) return; 

  
                   //////////////////////////////////////////////////////////////////////////////////////
                   //                    REPORT FRONT LEFT STRUT
                   ///////////////////////////////////////////////////////////////////////////////////////
		   
		   
		   // Blink Sport Light 1 time as group 4 times To indicate FL strut
                   blinkLights(SportLight, 1,300,4,1000);  

		   // See if abort desired
                   if(readDiagSelectorSwitch()!= NOCHANGE) return; 


		   // Display Final test results for this Strut
		   if (errorFL == NO_FAILURES){
                       setLED(LED1,D_GREEN);
		       setLights(TourLight,HIGH);
		       setLights(SportLight,HIGH);
                       delay(2500);
		   }else if (errorFL == MOTOR_COMMAND_FAILURE){
                      blinkBothLights(3,300,3,1000);
                      setLED(LED1,D_RED);

		   }else if (errorFL == SIGNAL_WIRE_FAILURE){ 
	              setLED(LED1,D_YELLOW);
 	              for(int i=0;i<8;i++){
	                 setLights(SportLight,HIGH);
                         setLights(TourLight,LOW);
			 delay(250);
	                 setLights(SportLight,LOW);
                         setLights(TourLight,HIGH);
			 delay(250);
		      }
		      // Turn off Lights
	              setLights(SportLight,LOW);
                      setLights(TourLight,LOW);
		   }

		   // See if abort desired
		   if(readDiagSelectorSwitch()!= NOCHANGE) return; 
		   
                   delay(2000);

                   
                   //////////////////////////////////////////////////////////////////////////////////////
                   //                    REPORT FRONT Right STRUT
                   ///////////////////////////////////////////////////////////////////////////////////////
		   // Turn off Lights
	           setLights(SportLight,LOW);
                   setLights(TourLight,LOW);
		   
                   //SET LED 2 to White to indicate testing Front Right Strut 
                   setLED(LED2,D_WHITE);
		   
		   // Blink Sport Light 2 time as group 3 times To indicate FR strut  
                   blinkLights(SportLight, 2,300,4,1000);  

		   // See if abort desired
                   if(readDiagSelectorSwitch()!= NOCHANGE) return; 


		   // Display Final test results for this Strut
		   if (errorFR == NO_FAILURES){
                       setLED(LED2,D_GREEN);
		       setLights(TourLight,HIGH);
		       setLights(SportLight,HIGH);
                       delay(2500);
		   }else if (errorFR == MOTOR_COMMAND_FAILURE){
                      blinkBothLights(3,300,3,1000);
                      setLED(LED2,D_RED);
                      delay(2500);

		   }else if (errorFR == SIGNAL_WIRE_FAILURE){ 
  	              setLED(LED2,D_YELLOW);
		      for(int i=0;i<8;i++){
	                 setLights(SportLight,HIGH);
                         setLights(TourLight,LOW);
			 delay(250);
	                 setLights(SportLight,LOW);
                         setLights(TourLight,HIGH);
			 delay(250);
		      }
		      // Turn off Lights
	              setLights(SportLight,LOW);
                      setLights(TourLight,LOW);
		   }

		   // See if abort desired
		   if(readDiagSelectorSwitch()!= NOCHANGE) return; 
		   
                   delay(2000);
  



                   //////////////////////////////////////////////////////////////////////////////////////
                   //                    REPORT REAR LEFT STRUT
                   ///////////////////////////////////////////////////////////////////////////////////////
		   // Turn off Lights
	           setLights(SportLight,LOW);
                   setLights(TourLight,LOW);
		   
                   //SET LED 3 to White to indicate testing Front Left Strut 
                   setLED(LED3,D_WHITE);
		   
		   // Blink Sport Light 3 time as group 3 times To indicate RL strut 
                   blinkLights(SportLight, 3,300,3,1000); 

		   // See if abort desired
                   if(readDiagSelectorSwitch()!= NOCHANGE) return; 


		   // Display Final test results for this Strut
		   if (errorRL == NO_FAILURES){
                       setLED(LED3,D_GREEN);
		       setLights(TourLight,HIGH);
		       setLights(SportLight,HIGH);
                       delay(2500);
		   }else if (errorRL == MOTOR_COMMAND_FAILURE){
                      blinkBothLights(3,300,3,1000);
                      setLED(LED3,D_RED);
                      delay(2500);

		   }else if (errorRL == SIGNAL_WIRE_FAILURE){
 	              setLED(LED3,D_YELLOW); 
		      for(int i=0;i<8;i++){
	                 setLights(SportLight,HIGH);
                         setLights(TourLight,LOW);
			 delay(250);
	                 setLights(SportLight,LOW);
                         setLights(TourLight,HIGH);
			 delay(250);
		      }
		      // Turn off Lights
	              setLights(SportLight,LOW);
                      setLights(TourLight,LOW);
		   }



		   // See if abort desired
		   if(readDiagSelectorSwitch()!= NOCHANGE) return; 
		   
                   delay(2000);
  

                   //////////////////////////////////////////////////////////////////////////////////////
                   //                    REPORT REAR RIGHT STRUT
                   ///////////////////////////////////////////////////////////////////////////////////////
		   // Turn off Lights
	           setLights(SportLight,LOW);
                   setLights(TourLight,LOW);
		   
                   //SET LED 1 to White to indicate testing Rear Right Strut 
                   setLED(LED4,D_WHITE);
		   
		   // Blink Sport Light 4 time as group 3 times To indicate RR strut 
                   blinkLights(SportLight, 4,300,3,1000);  

		   // See if abort desired
                   if(readDiagSelectorSwitch()!= NOCHANGE) return; 

		   // Display Final test results for this Strut
		   if (errorRR == NO_FAILURES){
                       setLED(LED4,D_GREEN);
		       setLights(TourLight,HIGH);
		       setLights(SportLight,HIGH);
                       delay(2500);
		   }else if (errorRR == MOTOR_COMMAND_FAILURE){
                      blinkBothLights(3,300,3,1000);
                      setLED(LED4,D_RED);
                      delay(2500);

		   }else if (errorRR == SIGNAL_WIRE_FAILURE){
 	              setLED(LED4,D_YELLOW); 
		      for(int i=0;i<8;i++){
	                 setLights(SportLight,HIGH);
                         setLights(TourLight,LOW);
			 delay(250);
	                 setLights(SportLight,LOW);
                         setLights(TourLight,HIGH);
			 delay(250);
		      }
		      // Turn off Lights
	              setLights(SportLight,LOW);
                      setLights(TourLight,LOW);
		   }
		    
                   delay(2000);
          }
          return;
}

//**************************************************************************
// this routine converts three seperate RGB values in to a correctly encoded RGB integer
//**************************************************************************
long rgbEncode(int red,int green,int blue){
 long rgb;

 //if (red > 255) red=255;
 //if (green > 255) green=255;
 //if (blue > 255) blue=255;

 rgb = red*65536 + green*256 + blue;

 return (rgb);
 
} 



/****************************************************************************************
 *
 *
 * Routine to read int from EProm
 *
 *
 ****************************************************************************************/
int eepromReadInt(int address){
   int value = 0x0000;
   value = value | (EEPROM.read(address) << 8);
   value = value | EEPROM.read(address+1);
   return value;
}

/****************************************************************************************
 *
 *
 * Routine to Write int to EProm
 *
 *
 ****************************************************************************************/
void eepromWriteInt(int address, int value){
   EEPROM.write(address, (value >> 8) & 0xFF );
   EEPROM.write(address+1, value & 0xFF);
}

//*******************************************************************************************
//
// Description:
// Takes the current LED color array and pushes it out & then waits a certain amount of time 
// for the frame to latch and display.
//
//********************************************************************************************
//
// This routine postLed() was originally written by Nathn Seidle/Sparkfun Electronics 2011 as 
// as part of some sample opensource for demonstrating how to use WS2801 strip LEDS. This is his
// open source requirement shared under the following statement from Nathan:
//  Nathan SeidleSparkFun Electronics 2011  " This code is public domain but you buy me a beer
//  "  if you use this and we meet someday (Beerware license)."
//  " 
// 
//  Nathan - I'll make it two - duke3k. It would be great to meet.
//
//
//********************************************************************************************
void postLED(void){
  //Each LED requires 24 bits of data
  //MSB: R7, R6, R5..., G7, G6..., B7, B6... B0 
  //Once the 24 bits have been delivered, the IC immediately relays these bits to its neighbor
  //Pulling the clock low for 500us or more causes the IC to post the data.

  for(int LED_number = 0 ; LED_number < NUMBER_LEDS ; LED_number++) {
    long this_led_color = LED[LED_number]; //24 bits of color data

    for(byte color_bit = 23 ; color_bit != 255 ; color_bit--) {
      //Feed color bit 23 first (red data MSB)
      
         
      long mask = 1L << color_bit;
      //The 1'L' forces the 1 to start as a 32 bit number, otherwise it defaults to 16-bit.
      
      digitalWrite(CKI, LOW); //Only change data when clock is low
      
      if(this_led_color & mask) 
        digitalWrite(SDI, HIGH);
      else
        digitalWrite(SDI, LOW);
  
      digitalWrite(CKI, HIGH); //Data is latched when clock goes high
    }
  }

  //Pull clock low to put LED String into reset/post mode
  digitalWrite(CKI, LOW);
  delayMicroseconds(1000); //Wait for 500us to go into reset
}



