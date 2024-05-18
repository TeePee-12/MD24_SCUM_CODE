#include <FlexCAN.h>
#include <SPI.h>
#include <SD.h>
#include <arduino-timer.h>

//-------------- Pin Definitions - Signals Internal to SCUM UNIT ---------------------------------

//      CAN_TX = 3; // managed by flexcan library
//      CAN_RX = 4; // managed by flexcan library
#define R2D_SIGNAL 5 //Constant R2D signal from SCUM Logic
#define BSPD_TRIP 6 //High on Shutdown Circuit Trip/Fault From SCUM Logic Output
#define IMD_TRIP 7 //High on Shutdown Circuit Trip/Fault From SCUM Logic Output
#define AMS_TRIP 8 //High on Shutdown Circuit Trip/Fault From SCUM Logic Output
#define IMD_CAN_SIG 9 //IMD trip output for use with the CANBUS IMD  to SCUM Locig Input
//      CS = 10; // managed by SD CARD library
//      DOUT = 11; // managed by SD CARD library
//      DIN = 12; // managed by SD CARD library
//      SCK = 13; // managed by SD CARD library

//-------------- Pin Definitions - Signals External to SCUM UNIT ---------------------------------
#define R2D_BUZZER 0 // Output signal to drive the buzzer
#define ON_SWITCH 1 // From Dashboard On Switch
#define START_BUT 2 // From Dashboard Start Button

#define APPSA A0 // SCUM_D10
#define APPSB A1 // SCUM D_09
#define SCUM_D08 16 //
#define SCUM_D07 17 //
#define SCUM_D06 18 //
#define SCUM_D05 19 // 
#define SCUM_D04 20 //
#define SCUM_D03 21 //
#define CURRENT_SENSOR A8 // Analog Current Signal - Post input filtering
#define BRAKE_PRESSURE A9 // Analog Brake Pressure Signal - Post input filtering

//-----------/Pedal Trace Initializations/---------------------------------

#define APPS_OUT A14 //DAC output

#define APPS_A_MIN 845
#define APPS_A_MAX 905
#define APPS_B_MIN 310
#define APPS_B_MAX 420
#define BPS_MIN  160 
#define BPS_TRIP 650 //Approx 10% brake force - Set this value to be much lower than the BSPD trip point
#define BPS_MAX 1600 // Re-checkk this value after brake system finsihed
volatile bool BRAKE_PLAUSABILITY_CHECK; //true = error, false = OK
volatile bool APPS_ERROR;
volatile int APPS_POSITION; // This gets written to an analog pin to relay pedal position
void pedalRead(); // Check all conditions for pedal traces and relay to output to MCU

//--------------/Other global variables/------------------------------------

volatile int BUZZER_TIMER = 0; // Used for timng the 2 second R2D buzzing
volatile bool R2D = false; // Boolean of the R2D state

void R2D_ISR();

//----------------/SETUP/---------------------------------------------------------------------
void setup() {
  
  APPS_POSITION = 0;
  R2D = false;
  pinMode(R2D_SIGNAL, INPUT_PULLDOWN);
  pinMode(R2D_BUZZER, OUTPUT);
  digitalWrite(R2D_BUZZER, LOW);

  pinMode(APPSA, INPUT);
  pinMode(APPSB, INPUT);
  pinMode(13, OUTPUT);
  pinMode(BRAKE_PRESSURE, INPUT);
  pinMode(APPS_OUT, OUTPUT);
  
  attachInterrupt(R2D_SIGNAL, R2D_ISR , CHANGE); // Interrupt on R2D to start the buzzer

  Serial.begin(9600);//initiate the serial, wait for it to be available before continuing. Timeout of 5 second if not available.
  while(!Serial && (millis() < 5000) ){
    delay(1);
  }
  if(Serial){
    Serial.print("HELLOWORLD\n");
  }
}
//------------------END SETUP--------------------------------------------------------------------------------



//----main loop----------------------------------------------------------------------------------------------
void loop(){
  delay(10);

  // Serial.print("Start Loop");
  // Serial.print(" |  R2D Status is: ");
  // Serial.print(R2D);
  // Serial.print(" | R2D Signal Input is: ");
  // Serial.print(digitalRead(R2D_BUZZER));
  // Serial.print("\n");


  pedalRead();


  // R2D is required to meet FSAE Rule EV10.4.2, EV.10.4.3
  while (R2D){
    //R2D BUZZER - FSAE RULE EV.10.5
    if ((millis() - BUZZER_TIMER) > 2000){ 
      digitalWrite(R2D_BUZZER, LOW); //Kill the buzzer if 2 seconds has passed since it turned on.
    }
    pedalRead(); // Handle pedal box inputs and write the analog output to the MCU
  }


  // Write to zero for FSAE Rule EV10.4.2, EV.10.4.3
  //APPS_POSITION = 0;
  //analogWrite(APPS_OUT, APPS_POSITION);
}

void R2D_ISR(){
  if(digitalRead(R2D_SIGNAL) == 0){
    Serial.print("Not R2D \n");
    digitalWrite(R2D_BUZZER, LOW);
    R2D = false;
  }
  if(digitalRead(R2D_SIGNAL) == 1){
    Serial.print("R2D \n");
    digitalWrite(R2D_BUZZER, HIGH);
    BUZZER_TIMER = millis();
    R2D = true;
  }
}

void pedalRead(){
  delay(500);
  // Convert ADC level to a pedal travel percentage. Mapped to maximum and minimum values allowed for a correct pedal signal.
  register float APPS_A = (analogRead(APPSA));
  // Serial.print("APPSA = ");
  // Serial.print(APPS_A);
  // Serial.print(" | ");

  APPS_A = ((APPS_A-APPS_A_MIN)/(APPS_A_MAX-APPS_A_MIN))*100;
  register float APPS_B = analogRead(APPSB);

  // Serial.print("APPSB = ");
  // Serial.print(APPS_B);
  // Serial.print(" | ");

  APPS_B = ((APPS_B-APPS_B_MIN)/(APPS_B_MAX-APPS_B_MIN))*100;
  register int BPS = analogRead(BRAKE_PRESSURE);

  // Serial.print("BPS = ");
  // Serial.print(analogRead(BRAKE_PRESSURE));
  // Serial.print(" | ");
  // Serial.print("APPSA = ");
  // Serial.print(APPS_A);
  // Serial.print("% | ");
  // Serial.print("APPSB = ");
  // Serial.print(APPS_B);
  // Serial.print("% | ");

  
  // Check for APPS failure due to oper/short circuit - FSAE RULES T.4.2.10, T.4.3.3, T.4.3.4
  register bool out_of_bounds = ((APPS_A < 0 || APPS_A > 100) || (APPS_B < 0 || APPS_B > 100) || (BPS < BPS_MIN || BPS > BPS_MAX));
  
  // Check for APPS Plausability - FSAE RULES T.4.2.4, T.4.2.5
  register bool mismatch = abs(APPS_A-APPS_B) > 10; // MUST BE SET TO 10% FOR RULES COMPLIANCE
  
  // Brake System Encoder RSAE RULE EV.5.7
  if(BRAKE_PLAUSABILITY_CHECK){
    if( APPS_A<2 && APPS_B<2 && BPS < BPS_MIN+10 ){
      BRAKE_PLAUSABILITY_CHECK = false;
    }
  }else{
    BRAKE_PLAUSABILITY_CHECK = ((APPS_A>10 || APPS_B>10) && BPS>BPS_TRIP);
  }
  // Serial.print("missmatch = ");
  // Serial.print(mismatch);
  // Serial.print(" | out of bounds = ");
  // Serial.print(out_of_bounds);
  // Serial.print(" | Brake plausability fault = ");
  // Serial.print(BRAKE_PLAUSABILITY_CHECK);

  // Write global pedal error flag, update pedal trace output
  APPS_ERROR = (mismatch || out_of_bounds || BRAKE_PLAUSABILITY_CHECK);

  // Serial.print(" | APPS ERROR = ");
  // Serial.print(APPS_ERROR);
  
  if (APPS_ERROR){
    APPS_POSITION = 0;
    analogWrite(APPS_OUT, APPS_POSITION);
  }else{
    APPS_POSITION = APPS_B; //Set position to APPS_A
    analogWrite(APPS_OUT, APPS_POSITION);
  }

  // Serial.print(" | APPS OUT = ");
  // Serial.print(APPS_POSITION);
  // Serial.print("\n");
}
