// Just plot Pr sesnor
#include "RunningAverage.h"

RunningAverage myRA(10);
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
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.print("\nDemo ");
  Serial.println(__FILE__);
  Serial.print("Version: ");
  Serial.println(RUNNINGAVERAGE_LIB_VERSION);
  
  
  myRA.clear(); // explicitly start clean
  Serial.println("\n millSec \tGP cmH2o \t Dpff h20");

}

void loop() {
  // put your main code h1/.09ere, to run repeatedly:
  SV = analogRead(PS_50GP) * adc2V;
  vout = SV - Vos50;
  Pr50GP = (vout /.09)*10; //Vref*0.018=0.09; 0.018 is slope of voltage to pr graph (Kpa)
   myRA.addValue(Pr50GP);
   avgPr50 = myRA.getAverage();
  SV= analogRead(PS_02DP) * adc2V;
  vout=SV- Vos02;
  Pr02DP=(vout/1)*10;
  
  Serial.print(millis());Serial.print("\t");    Serial.print(avgPr50);Serial.print("\t");Serial.println(Pr02DP);
   delay(20);
   
}
