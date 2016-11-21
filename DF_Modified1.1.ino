  //*************************************************
//Common out define DEBUG when load to Mega board:
 #define DEBUG     // <--- this is the key line
 void dummy(){}
//*************************************************
 
/* 2015 CHESS Team - Vanderbilt Senior Design
Read Battery SOC
Reads the voltage drop on the current shunts and count amp hour to calculate the SOC of the lead acid battery
This method is not accurate, and is intended to be replaced with programming to control a TI BQ34Z110 chip
Feel free to contact me david.x.feng@vanderbilt.edu for clarification of the program!
Controls generator ignition and switch off
Reads power output from solar and wind sources using TI INA283 and INA282 current shunt monitoring chips
*/



#include <math.h>
#include "LiquidCrystal.h"
#include <Wire.h>
 

//Constants
#define RELAY_ON 1
#define RELAY_OFF 0
 

//Define Pin numbers for Processor Board

   

    //**************************************************
    // Arduino Digital I/O pin number for Arduino MEGA *
    //**************************************************
    //relay and powerswitch pin setup
    int ignitionswitch1 = 22;
    int ignitionswitch2 = 24;
    int batteryswitch = 26;
    int RelayStartbutton = 28;
    int RelayStopbutton = 30;
    int Powerswitch= 32;
    
    //LCD Din pins:
    int lc1 = 7;
    int lc2 = 6;
    int lc3 = 5;
    int lc4 = 4;
    int lc5 = 3;
    int lc6 = 2;
    
    //Butons for LCD Display ??
   
    int startButton = 12; 
    int stopButton = 11;
    int leftButton = 10;
    int rightButton = 9;

    //set flash LED PIN 
    int flashLEDPin = 13;
    int enableChargingPin = 8;
    
    //**************************************************
    // Arduino Analog I/O pin number for Arduino MEGA  *
    //**************************************************
    
    //  for dc analog voltage measurement
    int panelCurrentPin = 1;  
    int chargecontrCurrentPin = 0; 
    int BattShuntPin = 2;
    
    int panelVoltagePin = 7;
    int chargecontrVoltagePin = 6;
      
    // Pins for Battery Voltage Measurement
    int Battery1VoltagePin = 8;
    int Battery2VoltagePin = 9;
    int Battery3VoltagePin = 10;
    int Battery4VoltagePin = 11;

    int lastAnalogPinNum = 14;   // last analog pin number A15 used 
    

//*********************************************
// Coefficient of Voltage Measurement in mV   *
//*********************************************

    float kPanelVoltage  =  (220.0 / 15.0);   // ADC conversion (5.0 / 1023.0), "235 / 15" comes from the voltage divider, see schematic

    float kPanelCurrent =  2.0 ;              // Vout = Gain * Vin = Gain * I * Rs
                                              // I = Vout /(Gain * Rs); Rs = Vs/Is ; Vs=30V,Is=75mA, Gain=200 for INA283  
                                              // 2.0 =  30.0 / (0.0750 * 200.0); // 30A/75mV current shunt conversion, 200 V/V gain amplifier conversion (TI INA283)
   
    float refPanelVoltage = 2500.0;           // mV (INA283 refvoltage = (ref1 + ref2)/2.0)
 
    float kChargecontrVoltage  = 100.0 / 10.0;// = (110.0 / 10.0); // ADC conversion (5.0 / 1023.0), "110/10" comes from the voltage divider, see schematic
                                              //chargecontrVoltage = (ADCValue / 1023.0) * 5.0 * (110.0 / 10.0); // ADC conversion (5.0 / 1023.0), "110/10" comes from the voltage divider, see schematic
 
    float kChargecontrCurrent = 6.666 ;         // 6.66 = (50)/(100*.075) 50A/75mV current shunt conversion, 100 V/V gain amplifier conversion (TI INA286)
                                              // panelCurrent = (ADCValue / 1023.0) * 5.0 * 30.0 / 0.0750 / 100.0; // 30A/75mV current shunt conversion, 100 V/V gain amplifier conversion (TI INA286)
    
    float refChargecontrVoltage = 2500.0;     // (INA286 refvoltage = (ref1 + ref2)/2.0) mV

    float kShuntCurrent =  13.333;              // Battery Shunt Current Constant = (50/(50*.075  50A/75mV current shunt conversion, 50 V/V gain amplifier conversion (TI INA282)
    float refShuntVoltage = 2500.0;
       
//   Battery Voltage Measurement Coefficient
    float kBatteryVoltage1 = 10.0;            //setting to 10 times at Grove Votage Divider
    float kBatteryVoltage2 = 10.0;            // need calibrate 
    float kBatteryVoltage3 = 10.0;
    float kBatteryVoltage4 = 10.0;

    float batteryLowLimit  = 900.0  ; // mV
    float batteryHighLimit = 1300.0 ; //mV

    int stateOfCharge   = 0;   //0:no charge ; 1: charging
//**************************
// control variable        *
//**************************
 
 int AvgNum = 1000;   //read anolog pin average times
 
int numLoops = 0; // number of times code has gone through main loop (run generator start/stop sequence every 20 times)

float ADCValueBattery1 = 0.0;
float ADCValueBattery2 = 0.0;
float ADCValueBattery3 = 0.0;
float ADCValueBattery4 = 0.0;

float ADCReading[15];


// how many times the control system checks the button states before checking the generator
// increasing improves system responsiveness at the price of not updating soc variable as quickly
int BUTTONS_PER_GENERATOR = 100;

// interface pin setup (TODO: CHANGE AS NECESSARY)
LiquidCrystal lcd(lc1, lc2, lc3, lc4, lc5, lc6);

// output variables
int leftButtonState  = LOW;
int rightButtonState = LOW;
int startButtonState = LOW;
int stopButtonState  = LOW;

long lastLeftPressedTime = 0;
long lastRightPressedTime = 0;
long debounceDelay = 200;
int  displayType = 0;

int flashLEDState = LOW;
int enableCharging = 0;   //0:disable 1: enable
   


//state of charge in percentage
float soc;
  
unsigned int ADCValue;
double Vcc;
float totalBattVolt;

/* 2014 TEAM
//initialize voltage and for shunt #1 (input from battery charger)
int batMonPin0 = 0; // input pin
float voltageshunt1 = 0; // variable to hold the voltage drop for shunt 1
//initialize voltage and for shunt #2 (input from load)
int batMonPin1 = 1; // input pin
float voltageshunt2 = 0; // variable to hold the voltage drop for shunt 1
//current from voltage 1
float current1 = 0;
float amphour1 = 0;
//current from voltage 2
float current2 = 0;
float amphour2 = 0;
//netamphour
float netamp = 0;
float netamphour = 0;
*/


// dc power measurement

float panelVoltage = 0.0;
float panelCurrent =  0.0;
float BattShuntCurrent = 0.0;

float solarPowerOutput =  0.0;
float chargecontrVoltage =  0.0;
float chargecontrCurrent =  0.0;
float chargecontrPowerOutput =  0.0;
float windPowerOutput =  0.0;


float batteryVoltage1 =  0.0;
float batteryVoltage2 =  0.0; 
float batteryVoltage3 =  0.0; 
float batteryVoltage4 =  0.0; 


//sampling
boolean freshstart = 1;
int sample = 0;
unsigned long time;
//improve accuracy of the A/D conversion




void setup() {
  
        //-------( Initialize Pins so relays are inactive at reset)----
        digitalWrite(ignitionswitch1, RELAY_OFF);
        digitalWrite(ignitionswitch2, RELAY_OFF);
        digitalWrite(batteryswitch, RELAY_OFF);
        digitalWrite(RelayStartbutton, RELAY_OFF);  
        digitalWrite(RelayStopbutton, RELAY_OFF);  
        
        pinMode(ignitionswitch1, OUTPUT);
        pinMode(ignitionswitch2, OUTPUT);
        pinMode(batteryswitch, OUTPUT);
        pinMode(RelayStartbutton, OUTPUT);
        pinMode(RelayStopbutton, OUTPUT);
 
        // initialize digital pin 13 as an output.
        pinMode(flashLEDPin, OUTPUT);  
      
        delay(4000); //Check that all relays are inactive at Reset
        
        Serial.begin(9600); // open the serial port at 9600 baud
        pinMode(leftButton, INPUT);
        pinMode(rightButton, INPUT);
        lcd.begin(20, 4);
        lcd.clear();
}
 

//Read Analog to Digital Voltage Value of Analog pins from number of 0 to endpin

void readADCValues( float (& adcReading) [15],  int endPin  , int AvgNum  )
{ 
    long sum [endPin];
    long reading[endPin];
    for (int k= 0; k < endPin ; k++)
    {
       sum[k] = 0;
       reading[k] = 0;
    }
   //read adc value for all pins
    for (int j = 0; j < AvgNum; j ++)
    {  
        for(int i = 0; i<endPin; i++)
        {
           reading[i] = analogRead(i);          
           sum[i] = reading[i] + sum[i];
        }        
         delay(2);  
     } 
     float v ;
    for (int k = 0; k < endPin ; k++)
    {
        v = sum[k]/ AvgNum;
        adcReading[k] = v * 4980.0 / 1023.0   ;   
    }
}

/**********************************************************************
         Read Digital Pins 
**********************************************************************/
void readDigitalPins()
{
  //Read Digital Pins
    // read button states
    leftButtonState  = digitalRead(leftButton);
    rightButtonState = digitalRead(rightButton);
    startButtonState = digitalRead(startButton);
    stopButtonState  = digitalRead(stopButton);
    enableCharging = digitalRead(enableChargingPin);
    
    #ifdef DEBUG
        Serial.print("\nState Of Charging:  ");
        Serial.print( stateOfCharge );  
     if(  stateOfCharge == 1)Serial.print(": charging ") ; else Serial.print(": not charging ");    
        Serial.print("\nBottonState:Start,Stop,Left,Right; ");
      // start generator if start button is pressed
    if (  enableCharging   == HIGH)Serial.print(" High "); else  Serial.print(" Low "); 
    if (  startButtonState == HIGH)Serial.print(" High "); else  Serial.print(" Low "); 
    if (  stopButtonState  == HIGH)Serial.print(" High "); else  Serial.print(" Low "); 
    if (  leftButtonState  == HIGH)Serial.print(" High "); else  Serial.print(" Low "); 
    if (  rightButtonState == HIGH)Serial.print(" High "); else  Serial.print(" Low ");  
    #endif
 
}

/**********************************************************************
         Read Analog Pins 
**********************************************************************/
void readAnalogPins()
{    
     //establish Vcc
     // Vcc = readVcc()/1000.0;  //???
      
      //Read all analog ADC input,
       readADCValues(  ADCReading, lastAnalogPinNum, AvgNum);
    
    
      // MEASURE SOLAR POWER OUTPUT
      // Measures DC output directly from solar panels
      // Voltage comes from a voltage divider
      // Current comes from current shunt/shunt monitor chip (INA283)
      
      //ADCValue = readVoltageAvg(panelVoltagePin,500);
      //panelVoltage = (ADCValue / 1023.0) * 5.0 * (235.0 / 15.0); // ADC conversion (5.0 / 1023.0), "235 / 15" comes from the voltage divider, see schematic
    
     //  ADCValue = readVoltage(chargecontrVoltagePin);
       chargecontrVoltage = (ADCReading[chargecontrVoltagePin] * kChargecontrVoltage); // ADC conversion (5.0 / 1023.0), "110/10" comes from the voltage divider, see schematic
    
       panelVoltage = ADCReading[panelVoltagePin] * kPanelVoltage;
      #ifdef DEBUG
        Serial.print("\nSolar Panel and Charge Controller Voltage:  ");
        Serial.print(panelVoltage);
        Serial.print("  ");
        Serial.print(chargecontrVoltage);
        Serial.print(" mV");
        #endif 
        
     //Get Current in [mA]
          panelCurrent       = (ADCReading[panelCurrentPin] - refPanelVoltage) * kPanelCurrent  ; //      30.0 / 0.0750 / 200.0; // 30A/75mV current shunt conversion, 200 V/V gain amplifier conversion (TI INA283)
          chargecontrCurrent = (ADCReading[chargecontrCurrentPin] - refChargecontrVoltage) * kChargecontrCurrent  ; // 1000.0 to Volt., 30A/75mV current shunt conversion, 100 V/V gain amplifier conversion (TI INA286)
          BattShuntCurrent   = (ADCReading[BattShuntPin] - refShuntVoltage) * kShuntCurrent  ; //    30.0 / 0.0750 / 200.0; // 30A/75mV current shunt conversion, 200 V/V gain amplifier conversion (TI INA283)
    
      
      #ifdef DEBUG
        Serial.print("\nSolar Panel,Control and Battery Current:");
        Serial.print(panelCurrent);
        Serial.print(", ");
        Serial.print(chargecontrCurrent);
        Serial.print(", ");
        Serial.print(BattShuntCurrent);
        Serial.print(" mA");
        
      #endif 
       
    
      
      solarPowerOutput = (panelVoltage / 1000.0) * (panelCurrent / 1000.0)  ; // power is product of voltage and current, 
       #ifdef DEBUG
          Serial.print("\nSolar Panel Power: ");
          Serial.print(solarPowerOutput);
          Serial.print(" W");
       #endif
       
      // MEASURE WIND POWER OUTPUT
      // Measures DC output from charge controller (solar + wind generation)
      // Voltage is assumed to be 48V
      // Current comes from current shunt/shunt monitor (INA282)
      // DC Wind power is calculated by subtracting the solar value from the charge controller total
      
     
        
      chargecontrPowerOutput = (chargecontrVoltage/1000.0) *  (chargecontrCurrent /1000.0)  ; // power is product of voltage and current
      windPowerOutput = chargecontrPowerOutput - solarPowerOutput; // wind power is approx. total charge controller output minus the solar generation
       #ifdef DEBUG
          Serial.print("\nCharge Controller Power: ");
          Serial.print(chargecontrPowerOutput);
          Serial.print(" W");
          Serial.print("\nWind Power: ");
          Serial.print(windPowerOutput);
          Serial.print(" W");
        #endif 
      
    
      //Measure battery voltage
      batteryVoltage1 = ADCReading[Battery1VoltagePin] * kBatteryVoltage1 - 540.00;  
      batteryVoltage2 = ADCReading[Battery2VoltagePin] * kBatteryVoltage2 - 540.00;
      batteryVoltage3 = ADCReading[Battery3VoltagePin] * kBatteryVoltage3 - 540.00; 
      batteryVoltage4 = ADCReading[Battery4VoltagePin] * kBatteryVoltage4 - 540.00;
    
      totalBattVolt =  batteryVoltage1 + batteryVoltage2 +batteryVoltage3 + batteryVoltage4;
      #ifdef DEBUG
          Serial.print("\nBattery Voltage: TEST Reading: \n total:");
          Serial.print(totalBattVolt);
          Serial.print(", ");
          Serial.print(batteryVoltage1);
          Serial.print(", ");
          Serial.print(batteryVoltage2);
          Serial.print(", ");
          Serial.print(batteryVoltage3);
          Serial.print(", ");
          Serial.print(batteryVoltage4);
          Serial.print("(mV) "); 
    
    #endif 

}

/**********************************************************************
         Main Loop of Program
**********************************************************************/
void loop() {
  #ifdef DEBUG
     Serial.print("\n ************* New Loop *****************");
  #endif
  // flash the LED on (HIGH is the voltage level)
  digitalWrite(flashLEDPin, flashLEDState);    
  if (flashLEDState == LOW)flashLEDState = HIGH;else flashLEDState = LOW;

  //Read All AnalogPins  
    readAnalogPins();
   
  //Read Digital Pins
    readDigitalPins();
  

  
//  2014 Code used to measure SOC
//String batteryState = "discharging"; // set to "discharging", "charging", or "full"
//peukert's law parameters ref: http://www.39pw.us/car/peukertEffect.html
float ratedcapacity = 120; //battery bank rated capacity in amp hour
float rval = 20; //hours used in stating capacity. Almost always 20 hours
float peukertexp = 1.15; //Peukert exponent for the battery
float param1 = log(rval/ratedcapacity); // natural log of (r/c)
float param2 = peukertexp * log(BattShuntCurrent); // peukertexp * natural log of (netamp)
float param3 = rval/ (exp(param2) *(peukertexp*param1)); // R/(EXP(param2)*EXP(peukertexp*param1)) = Hours
float param4 = param3*abs(BattShuntCurrent/1000)*3600; // param3*abs(netamp)*3600 = apparent capacity in amp second
float param5 = 100* (param4/ (ratedcapacity*3600));// peukerteffect in %
param5 = soc;
float remainingcharge = ratedcapacity*3600; // remaining charge in amp second




if ( stateOfCharge == 0 && ( totalBattVolt) < 46000.0 )
 {
     Start();
     stateOfCharge = 1;
     Serial.print("\n********** Start charging ... ********** SOC = ");
     Serial.print(stateOfCharge);
 }

//stop charging
if ( stateOfCharge == 1 && ( totalBattVolt) > 50000.0 )
  {
     Stop();
     stateOfCharge = 0;     
     Serial.print("\n********** STOP charging ... **********SOC = ");
     Serial.print(stateOfCharge);  
 }

 
  // start generator if start button is pressed
  if ( startButtonState == HIGH) 
  {
    // if (enableCharging == HIGH)
     //{  
        Start();
        stateOfCharge =1;
     /* }else
      {
       
        //show please enable charging switch on display
        Serial.print("\n********** Please Turn on Enable Charging switch ********* ");
       */ 
  
      //}
  }
  
  // stop generator if stop button is pressed
  if ( stopButtonState == HIGH) {
    Stop();
    stateOfCharge =0;
  }
  
  // if either button state is HIGH, we need to shift the display
  if (leftButtonState == HIGH) {
    lcd.clear();
    // check edge condition
    if (displayType == 0) {
      displayType = 3;
    }
    else {
      displayType--;
    }
  }
  
  if (rightButtonState == HIGH)
  {
    
    lcd.clear();
     if (displayType == 3) {
      displayType = 0;
    }
    else {
      displayType++;
         }
  }

  

  printDisplayInfo();

}


// START/STOP SEQUENCE

void Start(){
	lcd.clear();
	lcd.setCursor(0,0);
	lcd.print("CHESS CONTROL SYSTEM");
	lcd.setCursor(0,1);
	lcd.print("STARTING GENERATOR");
	// close Battery switch
	digitalWrite(batteryswitch, RELAY_ON);
	// open 1 and 2
        Serial.println("Turning on battery");
	digitalWrite(ignitionswitch1, RELAY_OFF);
	digitalWrite(ignitionswitch2, RELAY_OFF);
	digitalWrite(RelayStartbutton, RELAY_OFF);
  digitalWrite(RelayStopbutton, RELAY_OFF);
  
	// 2 second delay
	delay(1000);
	// Start generator by closing start switch
	digitalWrite(RelayStartbutton,RELAY_ON);
        Serial.println("Closing startbutton switch");
	// 3.5 second delay
	delay(2500);
	// Open start switch
	digitalWrite(RelayStartbutton,RELAY_OFF);
        Serial.println("Opening startbutton switch");
}

void Stop(){
	lcd.clear();
	lcd.setCursor(0,0);
	lcd.print("CHESS CONTROL SYSTEM");
	lcd.setCursor(0,1);
	lcd.print("STOPPING GENERATOR");
	//close battery switch
	digitalWrite(batteryswitch, HIGH);
	// Turn generator off //Close (Ground) ignition switches
	digitalWrite(ignitionswitch1, HIGH);
	digitalWrite(ignitionswitch2, HIGH);
	//3 second delay
	delay(3000);
	// Reset all switches to standby state
	digitalWrite(ignitionswitch1, LOW);
	digitalWrite(ignitionswitch2, LOW);
	digitalWrite(RelayStartbutton, LOW);
	// open battery switch
	digitalWrite(batteryswitch, LOW);
	lcd.clear();
	printDisplayInfo();
}



void printDisplayInfo()
{
	lcd.setCursor(0,0);
	lcd.print("CHESS CONTROL SYSTEM");
	lcd.setCursor(0,1);

switch (displayType) {
    case 0: // current solar power output
		lcd.print("GEN:");
            if (stateOfCharge = 0)
                {lcd.print("ON "); }
            if (stateOfCharge = 1)
                 {lcd.print("OFF"); }
        lcd.setCursor(8,1);
        lcd.print("Batt");
        lcd.print(soc);
        lcd.print("%");
        lcd.setCursor(12,2);
        lcd.print(totalBattVolt/1000);
     break;
     case 1: // current solar power output
		lcd.print("PWR:");
                lcd.print((panelCurrent/1000)*(panelVoltage/1000));
		lcd.setCursor(10,1);
                lcd.print("VLT:");
		lcd.print(panelVoltage/1000);
                lcd.setCursor(10,2);
                lcd.print("CUR:");
                lcd.print(panelCurrent/1000);

               
     break;
     
 
     case 2: // current wind of battery
		lcd.print("PWR:");
                lcd.print(windPowerOutput);
		lcd.setCursor(10,1);
                lcd.print("VLT:");
		lcd.print((chargecontrVoltage - panelVoltage)/1000);
                lcd.setCursor(10,2);
                lcd.print("CUR:");
                lcd.print((chargecontrCurrent - panelCurrent)/1000);

      
     break;
     case 3: //  gen what kind of power? gas? solar/wind?
		lcd.print("Status: "); 
            if (stateOfCharge = 0)
                {
                lcd.print("ON "); 
                }
            if (stateOfCharge = 1)
                 {
                 lcd.print("OFF");
                 }         
     break;
	}

	// display immediately adjacent interface options
	lcd.setCursor(0,3);
	lcd.print((char)127); // left arrow sign
	// use displayType as switch-case. Due to width of LCD screen, make sure that printed statements are
	// length 18 (since two characters are reserved for left and right arrow signs)

	switch(displayType) {
		case 0: // LEFT: TYPE OF POWER, RIGHT: WIND PWR OUTPUT
		lcd.print("MAIN slr wnd gen");  
                lcd.setCursor(20,0);
                lcd.print("1");

		break;
		case 1: // LEFT: SOLAR PWR OUTPUT, RIGHT: SOC
		lcd.print("main SLR wnd gen");
                lcd.setCursor(20,0);
                lcd.print("2");
		break;
		case 2: // LEFT: WIND PWR OUTPUT, RIGHT: TYPE OF POWER
		lcd.print("main slr WND gen");
                lcd.setCursor(20,0);
                lcd.print("3");
		break;
		case 3: // LEFT: SOC, RIGHT: SOLAR PWR OUTPUT
		lcd.print("main slr wnd GEN");
                lcd.setCursor(20,0);
                lcd.print("4");
		break;
	}

	lcd.print((char)126); // right arrow sign
}





long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1) || defined(__AVR_ATmega2560__) //CHANGE BY NICK
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
  uint8_t low = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both
  long result = (high<<8) | low;
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}
 

