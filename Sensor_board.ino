
/*****************************************************
*   Sofware to control xxx sensor reading
*   using a Atmega 32U4 processor
*
*
*   Version 2024-05-05 Wishtech
******************************************************/
#include "AD524X.h"   // Resistor ladder AD5242 (2 x digital potentiometer)

// Debug
#define DEBUG

// Define I/O-ports
#define LED_Red 15
#define LED_Green 16
#define LED_Blue 17
#define LED_ON false
#define LED_OFF true

//****************************
#define CalibBitsLowLimit  -20        // 5 volt / 1024 =  4.88 mV/bit, -0.1V -> 20.48 bits use 20 bits
#define CalibBitsHighLimit  20
#define CalibBitsDummy 100            // Just for start value
#define AREF  5                       // Analoge referese valu, internal 5V
#define ResNet1 0                     // Resistor net 1
#define ResNet2 1                     // Resistor net 2
#define ADCresolution 1024            // Atmega 32U4 has 10 bits resolution
// Analoge ports
#define WBrefRAW A0                   // Wheatston bridge reference RAW value (bits)
#define NOxRAW A6                     // Nox sensor RAW value (bits)
#define TempRAW A7                    // Temperature sensor RAW value (bits)

#define SensorTempOk 200              // 200 degrees Celsius
#define ResLadderStartValue 0x80      // Startup 1 M ohm

#define ComSpeed 19200                // Communication speed 19200 bps

#define FlashTime 500                 // Time LED on and LED off 

AD524X AD01(0x2C);                    //  Make an instance. AD0 & AD1 == GND makes I2C address 0x2C

bool LED_GreenFlashing = false;       // Used to LED flasing LED
bool GreenLight = false;              // Used to toggle LED flashing
bool StreamData = false;              // "Enable" for streaming data

unsigned long StartTimer;             // Timer used for LED flashing

float SensorTempC = 0;                // Temperature sensor value in Celcius

/*****************************************************
*   Prepare for Main loop
******************************************************/
void setup() {

  // Setup serial port
  Serial.begin(ComSpeed);
 
  // Setup I2C
  Wire.begin();
  Wire.setClock(400000);        // I2C clock speed

  // Setup I/O-pins
  pinMode(LED_Red, OUTPUT);
  pinMode(LED_Green, OUTPUT);
  pinMode(LED_Blue, OUTPUT);

  // Setup LED start light
  digitalWrite(LED_Green, LED_OFF);
  digitalWrite(LED_Blue, LED_OFF);
  digitalWrite(LED_Red, LED_ON);

  #ifdef DEBUG
    Serial.println("Waiting for temperature rise");
  #endif
  // Wait for temperature to rise
  while( SensorTempC < SensorTempOk){    
    SensorTempC = (int)(((((AREF*(float)analogRead(TempRAW))/(float)ADCresolution) * 1000.0/23.0) - 100.0)/0.385);
  }

  #ifdef DEBUG
    Serial.print("Sensor temp is ");
    Serial.println(SensorTempC);
  #endif

  // New LED config
  digitalWrite(LED_Red, LED_OFF);
  digitalWrite(LED_Blue, LED_ON);
  
  int GND_Bits = analogRead(WBrefRAW);
  float CalibBits = CalibBitsDummy;             // Start value (illegal)
  
  unsigned int ResLadderValue = 0x100;          // Set startup resistance 1M ohm   
  AdjustNet(ResLadderValue);                    // Config resistance ladder
  unsigned int ResAddSubLadderValue = ResLadderValue;        // 
 
  #ifdef DEBUG
    Serial.print("Start calibrate, ResLadder = ");
    Serial.println(ResLadderValue);
  #endif

  // Calibrate
  while(CalibBits <= CalibBitsLowLimit || CalibBits >= CalibBitsHighLimit){
    CalibBits = GND_Bits - analogRead(NOxRAW);
    if(CalibBits > CalibBitsHighLimit){         // To High resistance
      ResLadderValue = ResLadderValue - ResAddSubLadderValue + ResAddSubLadderValue/2;
      ResAddSubLadderValue /=2;
    }else{                                      // To Low resistance  
      ResLadderValue = ResLadderValue + ResAddSubLadderValue/2;
      ResAddSubLadderValue /=2;
    }
  AdjustNet(ResLadderValue);
  #ifdef DEBUG
    Serial.print("ResLadder = ");
    Serial.println(ResLadderValue, HEX);
  #endif
  delay(1000);
  }
  digitalWrite(LED_Blue, LED_OFF);
  SensorTempC = (int)(((((AREF*(float)analogRead(TempRAW))/(float)ADCresolution) * 1000.0/23.0) - 100.0)/0.385);
  Serial.println(SensorTempC);
  Serial.println(AREF*(float)analogRead(NOxRAW)/1024.0,2);
  LED_GreenFlashing = true;
  StartTimer = millis();
}


/*****************************************************
*   Main loop
******************************************************/

void loop() {

  FlashLED(LED_GreenFlashing);            // If enabled flash led
    
  if(Serial.available()){                 // Check for command
    switch(Serial.read()){
      case 'M':                           // Command = Messsure
        LED_GreenFlashing = false;        // Stop LED flashing
        digitalWrite(LED_Green, LED_ON);  // Continuous light
        StreamData = true;                // Ok to stream data
        break;
      case 'S':
        StreamData = false;               // Command = Stop streaming
        LED_GreenFlashing = true;         // Enable LED flashing
        break;
      default:
        // Not a valid command, here you can send a error message to host
        break;
    } 
  }

  if(StreamData){
    Serial.println(0.00488 * (float)analogRead(NOxRAW), 2);           //Two decimals  5 volt / 1024 =  4.88 mV/bit
    delay(500);                                                       // Wait for Host
  }   
}


/*'****************************************************
*   Controls the LED flashing
*
*   Input: none
*   Return: none
*******************************************************/

void FlashLED(bool Enable){
    if(Enable){                             // Flash allowed? 
    if(millis() - StartTimer > FlashTime){  // If timeout, toggle the LED
      StartTimer = millis();
      if(GreenLight == false){
        digitalWrite(LED_Green, LED_ON);
        GreenLight = true;  
      }else{
        digitalWrite(LED_Green, LED_OFF);
        GreenLight = false;
      }
    }
  }
}

/*****************************************************
*   Controls the resistanse ladder (2 x digital potentiometer)
*
*   Input: 0 - 0x1FF ( 0 - 2 M Ohm)
*   Output: none
******************************************************/

void AdjustNet(unsigned int Value){
  
  if(Value & 0x100){
    AD01.write(ResNet1, 0xFF);      // Set Resistor net 1 to 1M ohm
  }else{
    AD01.write(ResNet1, 0x00);      // Set Resistor net 1 to 0 ohm
  }

  Value = Value & 0xFF;
  AD01.write(ResNet2, (byte)Value); // Set Resistor net 2 
}
