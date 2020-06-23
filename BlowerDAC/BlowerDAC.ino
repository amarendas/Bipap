// Blower is operated using potenio meter on A5
// Setting of MPC4725 is in the range of  0 -- 4095


#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include "RunningAverage.h"

Adafruit_MCP4725 BlowerDAC;
RunningAverage myRA(10);

const int pot1=A5;
const int PS_50GP = A1; // Pressure sensor gaure
const int PS_02DP = A0; //
const float Vref = 5.0; // Analog referance voltage
const float adc2V = 0.0049; //on scale of 0-5VDC
int N = 20; //no for samplese to calculate offser
float Vos50=0.227;
float Vos02=2.675;
float vout;
float SV ;
float Pr50GP; //P in cm of H2o
float avgPr50;// Pr avrage
float Pr02DP; //P in cm of H2o
int pot1val=0; //potentiometer value
int dacVal=0; 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println(" File Blower ADC");
  BlowerDAC.begin(0x64);
  myRA.clear(); // explicitly start clean
  Serial.println("\n millSec \tGP cmH2o \t Dpff h20");

}

void loop() {
  
  pot1val=analogRead(pot1);             // Read Pot Data
  dacVal=map(pot1val,0,1023, 0,  4095); //Scale 
  BlowerDAC.setVoltage(0,false);
  //Serial.print(pot1val);Serial.print("\t");Serial.println(dacVal);
  
  
  // Read Sensor Data
  SV = analogRead(PS_50GP) * adc2V;
  vout = SV - Vos50;
  Pr50GP = (vout /.09)*10; //Vref*0.018=0.09; 0.018 is slope of voltage to pr graph (Kpa)
   myRA.addValue(Pr50GP);
   avgPr50 = myRA.getAverage();
  SV= analogRead(PS_02DP) * adc2V;
  vout=SV- Vos02;
  Pr02DP=(vout/1)*10;
  
  //Serial.print(millis());
  Serial.print("\t");    Serial.print(avgPr50);Serial.print("\t");Serial.println(Pr02DP);
   delay(25);
   
}
