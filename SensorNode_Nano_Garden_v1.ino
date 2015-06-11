#include "LowPower.h"
#include <RHDatagram.h>
#include <RH_ASK.h>
#include <SPI.h> // Not actualy used but needed to compile

#include <SFE_BMP180.h>
#include <Wire.h>
SFE_BMP180 pressure;
#define ALTITUDE 1655.0 // Altitude of SparkFun's HQ in Boulder, CO. in meters


#define CLIENT_ADDRESS 7
#define SERVER_ADDRESS 2

const byte LED1_PIN=7;
const int A1_PIN=A0;
const int A2_PIN=A1;
const int A3_PIN=A2;
const int A4_PIN=A3;

const byte RHRX_PIN = 8; 
const byte RHTX_PIN = 4; 
const byte PWR1_PIN = 5;                                                                                                                                                                                                             ; 

int A1_VAL=0, A2_VAL=0, A3_VAL=0, A4_VAL=0;
double TdegF=0, PinHg=0, AinFT=0;

RH_ASK driver(2000,RHRX_PIN,RHTX_PIN);  //(speed, RXpin, TXpin, ptt;IN, inverted)

// Class to manage message delivery and receipt, using the driver declared above
RHDatagram manager(driver, CLIENT_ADDRESS);



void setup() {
  delay(500);
  Serial.begin(9600);
  delay(500);
  
  // start BMP180
//  if (pressure.begin())
//    Serial.println(F("BMP180 init success"));
//  else
//  {
//    Serial.println(F("BMP180 init fail\n\n"));
//    while(1); // Pause forever.
//  }
  
  // start RH Driver
  if (!manager.init())
    Serial.println("init failed");  /* DEFAULTS 434MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36  */
    
  pinMode(A1_PIN,INPUT_PULLUP); 
  pinMode(A2_PIN,INPUT_PULLUP); 
  pinMode(A3_PIN,INPUT_PULLUP); 
  pinMode(A4_PIN,INPUT); 
  pinMode(LED1_PIN,OUTPUT);
  pinMode(PWR1_PIN,OUTPUT); digitalWrite(PWR1_PIN, HIGH);

  LED_FLASH(6, 100, LED1_PIN);
  Serial.println(F("SETUP COMPLETE"));
  delay(500);
}



uint8_t data[] = "Hello World!";
//uint8_t buf[RH_ASK_MAX_MESSAGE_LEN]; // Dont put this on the stack:
 int wdtCount=0; 
 const int WDT_INTERVALS = 15; 
void loop() {
   delay(1);
   //Serial.print(F("cnt:"));
   //Serial.println(wdtCount);
   wdtCount++;
    if (wdtCount >= WDT_INTERVALS)
    {
      RH_SEND_DATA();
      wdtCount=0;
      
    }
   
//   A1_VAL = analogRead(A1_PIN);
//   A2_VAL = analogRead(A2_PIN);
//   A3_VAL = analogRead(A3_PIN);
//   A4_VAL = analogRead(A4_PIN);
//   Serial.print(A1_VAL);   Serial.print(F(" : "));  
//   Serial.print(A2_VAL);   Serial.print(F(" : ")); 
//   Serial.print(A3_VAL);   Serial.print(F(" : ")); 
//   Serial.println(A4_VAL);    
//   delay(100);
   
   
   LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); 
  // RH_SEND_MSG();
}




//-------------------
// RADIOHEAD SEND CODE 
//-------------------
void RH_SEND_MSG()
{ 
    Serial.print(F("SENDING RADIOHEAD: HELLO")); 
    const char *msg = "hello";
    driver.send((uint8_t *)msg, strlen(msg));
    driver.waitPacketSent();
    delay(200);
} 


void RH_SEND_DATA()
{ 
  const char *msg = "hello";
  char RHMsg[20];
  A1_VAL = constrain( map(analogRead(A1_PIN), 0, 1023, 0, 255), 0, 255);   //415 - 275 415,275
  A2_VAL = constrain( map(analogRead(A2_PIN), 0, 1023, 0, 255), 0, 255);   //415 - 275 415,275 
  A3_VAL = constrain( map(analogRead(A3_PIN), 0, 1023, 0, 255), 0, 255);   //415 - 275 415,275  
  A4_VAL = constrain( map(analogRead(A4_PIN), 0, 1023, 0, 255), 0, 255);   //415 - 275 415,275 
  
  //sprintf(RHMsg, "%d,%d,%d.", TdegF, PinHg, AinFT);
  sprintf(RHMsg, "%d,%d,%d,%d.", A1_VAL, A2_VAL, A3_VAL, A4_VAL);
  //sprintf(RHMsg, "%d.", A4_VAL);
 
  if (manager.sendto((uint8_t*)RHMsg, strlen(RHMsg), SERVER_ADDRESS))
  {
    Serial.print(F("Sending Data: "));
    Serial.println(RHMsg);
  }
   delay(1200);

}




//-------------------
// BMP_READ FUNC
//-------------------
void BMP_READ()
{ 
      char status;
      double T,P,p0,a;
    
      // Loop here getting pressure readings every 10 seconds.
    
      // If you want sea-level-compensated pressure, as used in weather reports,
      // you will need to know the altitude at which your measurements are taken.
      // We're using a constant called ALTITUDE in this sketch:
      
      Serial.println();
      Serial.print("provided altitude: ");
      Serial.print(ALTITUDE,0);
      Serial.print(" meters, ");
      Serial.print(ALTITUDE*3.28084,0);
      Serial.println(" feet");
      
      // If you want to measure altitude, and not pressure, you will instead need
      // to provide a known baseline pressure. This is shown at the end of the sketch.
    
      // You must first get a temperature measurement to perform a pressure reading.
      
      // Start a temperature measurement:
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.
    
      status = pressure.startTemperature();
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);
    
        // Retrieve the completed temperature measurement:
        // Note that the measurement is stored in the variable T.
        // Function returns 1 if successful, 0 if failure.
    
        status = pressure.getTemperature(T);
        if (status != 0)
        {
          // Print out the measurement:
          Serial.print("temperature: ");
          Serial.print(T,2);
          Serial.print(" deg C, ");
          Serial.print((9.0/5.0)*T+32.0,2);
          Serial.println(" deg F");
          
          
          
          // Start a pressure measurement:
          // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
          // If request is successful, the number of ms to wait is returned.
          // If request is unsuccessful, 0 is returned.
    
          status = pressure.startPressure(3);
          if (status != 0)
          {
            // Wait for the measurement to complete:
            delay(status);
    
            // Retrieve the completed pressure measurement:
            // Note that the measurement is stored in the variable P.
            // Note also that the function requires the previous temperature measurement (T).
            // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
            // Function returns 1 if successful, 0 if failure.
    
            status = pressure.getPressure(P,T);
            if (status != 0)
            {
              // Print out the measurement:
              Serial.print("absolute pressure: ");
              Serial.print(P,2);
              Serial.print(" mb, ");
              Serial.print(P*0.0295333727,2);
              Serial.println(" inHg");
    
              // The pressure sensor returns abolute pressure, which varies with altitude.
              // To remove the effects of altitude, use the sealevel function and your current altitude.
              // This number is commonly used in weather reports.
              // Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
              // Result: p0 = sea-level compensated pressure in mb
    
              p0 = pressure.sealevel(P,ALTITUDE); // we're at 1655 meters (Boulder, CO)
              Serial.print("relative (sea-level) pressure: ");
              Serial.print(p0,2);
              Serial.print(" mb, ");
              Serial.print(p0*0.0295333727,2);
              Serial.println(" inHg");
              

    
              // On the other hand, if you want to determine your altitude from the pressure reading,
              // use the altitude function along with a baseline pressure (sea-level or other).
              // Parameters: P = absolute pressure in mb, p0 = baseline pressure in mb.
              // Result: a = altitude in m.
    
              a = pressure.altitude(P,p0);
              Serial.print("computed altitude: ");
              Serial.print(a,0);
              Serial.print(" meters, ");
              Serial.print(a*3.28084,0);
              Serial.println(" feet");
              

              
              
            }
            else Serial.println("error retrieving pressure measurement\n");
          }
          else Serial.println("error starting pressure measurement\n");
        }
        else Serial.println("error retrieving temperature measurement\n");
      }
      else Serial.println("error starting temperature measurement\n");
      
      TdegF = (9.0/5.0)*T+32.0;
      PinHg = p0*0.0295333727;
      AinFT = a*3.2808;
      delay(5000);  // Pause for 5 seconds. 
}



void LED_FLASH(int freq, int duration, byte PIN)
{
  for(int k=0; k<freq; k++)
  {
    digitalWrite(PIN, HIGH);
    delay(duration);
    digitalWrite(PIN, LOW);
    delay(duration);
  }
}
