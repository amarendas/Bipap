
#include <Wire.h>
#include <PID_v1.h>
#include <Adafruit_MCP4725.h>
#include <LiquidCrystal.h>
#include <LiquidCrystal_I2C.h>

#include "RunningAverage.h"

Adafruit_MCP4725 dac;
LiquidCrystal_I2C lcd(0x27, 16, 2);
const int buzzer = 8;
const int blower_en = 31;
// FLOW & peak_detection variables
float flow=0;
float filtered_flow=0;
float alpha=0;
float sampling_time=0.005;  // in seconds 0.015
float fc=5;  //cutoff freq in Hz
int offset = 32000; // Offset for the sensor
float scale = 140.0; // Scale factor for Air and N2 is 140.0, O2 is 142.8
float positive_peak_flow=0;
float negative_peak_flow=0;
float previous_flow=0;
int peak_first=0;
bool ppf_detected = false;
bool npf_detected = false;

int Pot1Pin = A0;    // select the input pin for the potentiometer
int Pot2Pin = A4;    // select the input pin for the potentiometer
int Pot3Pin = A5;    // select the input pin for the potentiometer
int Pot1Value = 0;  // variable to store the value coming from the sensor.
int Pot2Value = 0;  // variable to store the value coming from the sensor.
int Pot3Value = 0;  // variable to store the value coming from the sensor.
int setPrAvg=0;
int cnt1=0;
int cnt2=0;
const int PS_50GP=A1; // presure senser pin
float avgPr50GP=0; float Vos50=0; // holds avrage pr, offset Voltage
float avgtestpr=4;
uint32_t counter;
const float Vref=5.0; // Analog referance voltage
const float adc2V=Vref/1024; //on scale of 0-5VDC

double PSet=4;//Set Pressure
double Pressureset=4;
double p=4;
double PSet_old=0;//Set Pressure
unsigned int psetI;//For lcd Display
unsigned int pactI;//For lcd Display
double Kp=120, Ki=100, Kd=0; // double Kp=200, Ki=100, Kd=50;
double PMax=0;
double PMin=0;
double Input,Output;
// double maxout=0;
// double minout=0;
PID myPID(&Input, &Output, &PSet, Kp, Ki, Kd, DIRECT);
//LiquidCrystal lcd(2,3,22,23,24,25,26,27,28, 29,30);
unsigned long t1;
unsigned long t4=0;
unsigned long t5=0;
unsigned long t6=0;
unsigned long int differenceT=0;
unsigned long int firstT=0;
unsigned long int currentT=0;
const int toggle = 31;// Ravinder
const int ON_OFF = 32;// Ravinder
int togglestate;  // Ravinder
int lasttogglestate = HIGH;  
int lastoffstate = HIGH; 
int lastonstate = LOW; 
unsigned long lastDebounceTime = 0; 
unsigned long lastoffdebouncetime=0; 
unsigned long lastondebouncetime=0; 
unsigned long debounceDelay = 50;
unsigned long debounceOFFDelay=34;
unsigned long debounceONDelay=50;
 
int ON_OFFstate; // Ravinder
double PrHigh=0;  // Ravinder
double PrLow = 0; // Ravinder
double settime=0;
int lcdPrHigh=4;
int lcdPrLow=4;
float delta=0;
unsigned long int timevalue=0;
int timt=0;
int duration=0;
int firsttime=0;
int ftime=1;
int setting=0;

// BIPAP VARIABLES
double IPAP=0;  // Ravinder
double EPAP = 5; // Ravinder
double BPM=20;  // Ravinder
double Itime=0;  // Ravinder
double Itrigger = 0.05*5; // Ravinder// 5% of EPAP
double Etime=0;
double TrigPr=0;
double Etrigger=0;
unsigned long T1=0;
unsigned long T2=0;
unsigned long Initial_T1=0;
unsigned long Initial_T2=0;
double BRate=0;
unsigned long temp=0;

void setup(void) {
  Wire.begin();
  Serial.begin(9600);
//  lcd.begin(16, 2);

  alpha=sampling_time / ((1/(2*3.14*fc)) + sampling_time);
 
  pinMode(blower_en,OUTPUT);
  //digitalWrite(blower_en, HIGH);
 // digitalWrite(blower_en, LOW);
  lcd.begin();
  lcd.print("    CPAP_V3");
  lcd.setCursor(0,1);
  lcd.print("Calib. Pr. Sen.");
  //Serial.println("Hello!");
  // For Adafruit MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
  // For MCP4725A0 the address is 0x60 or 0x61
  // For MCP4725A2 the address is 0x64 or 0x65
  dac.begin(0x60);
  Output=0;
  dac.setVoltage(Output, false); // Stop the compressor
  delay(1000);// Wait for the compressoer to completely stop
  calculate_offset(200);// Clculate offset based on N no of sample  

  // Configure the PID 
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0,4095);// Set the output Limits
  PSet_old=0;
  lcd.clear();
  pinMode(toggle,INPUT);// Ravinder
  digitalWrite(toggle, HIGH);
  pinMode(ON_OFF,INPUT);// Ravinder
  digitalWrite(ON_OFF, HIGH);
  pinMode(buzzer,OUTPUT);
 // digitalWrite(blower_en,LOW);
  t1=millis();
 }
void loop(void)
{
//  // TOGGLE SWITCH
/*  int reading= digitalRead(toggle);
  
  if(reading!=lasttogglestate)
    {
    lastDebounceTime = millis();
    }
 if ((millis() - lastDebounceTime) > debounceDelay) 
 {
   if (reading != togglestate) 
   {
      togglestate = reading;   */

      if(Serial.available()>0)
      {
        int bipap_start= Serial.parseInt();
        if(bipap_start==100)
        {
          BIPAP();
          
          }
        
        }
      
         togglestate= digitalRead(toggle);
         
         if (togglestate == LOW) 
        {
      //  Pot1Value = analogRead(Pot1Pin);  // commented for version3
      //  PrLow = map(Pot1Value, 0, 1023, 0, 30); // commented for version3
      //  lcdPrLow=PrLow; // commented for version3
        Pot2Value = analogRead(Pot2Pin);
        settime = map(Pot2Value, 0, 1023, 0, 45);
        duration=settime;
        
        firsttime=0;
        
        Pot3Value = analogRead(Pot3Pin);
        PrLow = map(Pot3Value, 0, 1023, 4, 20);
        lcdPrLow=PrLow;

        Pot1Value = analogRead(Pot1Pin);
        PrHigh = map(Pot1Value, 0, 1023, 4, 20);
        lcdPrHigh=PrHigh;

      //  write3LCD();
      
        setting=1;
         
         
             
     }
//  }
//  }
//  lasttogglestate=reading;
   
 // Pot1Value = analogRead(Pot1Pin);
 // PrHigh = map(Pot1Value, 0, 1023, 0, 30);
 // lcdPrHigh=PrHigh;
  

  // flow_reading();
  // peak_flow_detection();

  
      // ON_OFF SWITCH
 int readingonoff=digitalRead(ON_OFF);
   if(readingonoff!=lastoffstate)
   {
   lastoffdebouncetime=millis();
    }
    if(readingonoff!=lastonstate)
   {
   lastondebouncetime=millis();
    } 
 if ((millis() - lastoffdebouncetime) > debounceOFFDelay) 
    {
   if (readingonoff != ON_OFFstate) 
   {
      ON_OFFstate = readingonoff;
  
     }
  }
 if ((millis() - lastondebouncetime) > debounceONDelay) 
    {
   if (readingonoff != ON_OFFstate) 
   {
      ON_OFFstate = readingonoff;
  
     }
  }
  
  lastoffstate=readingonoff;
  lastonstate = readingonoff;
  normal_run();
  
///////////testing air leak/////////////


if(setting == 1)
 {
   p=PSet-1;
  if(pactI < p)
  {
    if(ftime==0)
    {
      ftime=1;
      t4=millis();
    }
    
     t5=millis();
     t6 = t5-t4; 
 if(t6 > 10000)
 { 
   lcd.clear();
    while(1)
    {
    tone(buzzer,2000);
    lcd.setCursor(0,0);
    lcd.print("AIR LEAKAGE");
    lcd.setCursor(0,1);
    lcd.print("FIX IT & RESTART");
    Output=0;
    dac.setVoltage(Output, false);
    }
  }
 }
  else
  {
    ftime=0;
    }
  
  //Serial.print(p,3);Serial.print("\t");Serial.print(ftime,3);Serial.print("\t");Serial.print(pactI,5);Serial.println("");
}
}
void normal_run() 
{
    if ((t1-millis())>1500){
       if(avgPr50GP < 0)  // added by ravinder
       {
        pactI=0;
        }
        else
        {
        pactI= avgPr50GP;
        }
    // WriteLCD();
     t1=millis();
    }
  //   Pot1Value = analogRead(Pot1Pin);// Read Potentiometer
  //  //Read_pot1(2);
  //  PSet = map(Pot1Value, 0, 1023, 0, 30);// Map max pr to max pot position
 //  ON_OFFstate = digitalRead(ON_OFF);
  // delay(10);
  
  if(ON_OFFstate == HIGH)
  {
     PSet=PrHigh; // added argument by ravinder
     firsttime=0;
     if(setting==1)
     {
      if((cnt1==10)||(cnt1>10)) // to slow the refresh rate of lcd between 5 to 10 per second
      {
       WriteLCD();
       cnt1=0;
      }
      cnt1=cnt1+1;
     }
  }
  if(ON_OFFstate == LOW)
 {  
  if(((PrHigh < PrLow)||(PrHigh == PrLow))&&(PrLow > 0)&& (PrHigh > 0)) //   if(((PrHigh < PrLow)||(PrHigh == PrLow))&&((PrLow > 0)||(PrLow == 0))&& ((PrHigh > 0)||(PrHigh == 0))) 
  {
     PSet = PrLow;
     lcdPrHigh=PrLow; 
     firsttime=0;
     timt=0.0;
    }
     if(setting==1)
     {
      if((cnt2==10)||(cnt2>10)) //  to slow the refresh rate of lcd between 5 to 10 per second
      {
      write2LCD();
      cnt2=0;
      }
      cnt2=cnt2+1;
     }
   
  if((PrHigh > PrLow)&&(PrLow > 0)&& (PrHigh > 0))
    {
       if(firsttime==0)
      { 
        delta = (PrHigh - PrLow);
        firsttime=1;
        firstT=millis();
        differenceT=0;
        timevalue = duration*60000; 
        PSet=PrLow + (delta*differenceT)/(timevalue); // 120000 edited for version 3 i.e. duration from pot value
       }
      else
       { 
        delta = (PrHigh - PrLow);
        currentT=millis();
        differenceT=currentT-firstT;
        timt = (double)differenceT/60000;
        //duration=2.0;
        duration=settime;
       if(differenceT > (timevalue))
        {
         PSet=PrHigh;
         timt= duration;
         //Serial.print(PSet,8);Serial.print("\t");Serial.print(digitalRead(ON_OFF));Serial.print("\t");Serial.print(PrLow);Serial.print("\t");Serial.print(PrHigh);Serial.print("\t");Serial.print("time :");Serial.print("\t");Serial.println(timt,3);   
        }
        else
        { 
        PSet = PrLow + (delta*differenceT)/(timevalue); // 45*60*1000= 2700000 added argument by ravinder
        //Serial.print(PSet,8);Serial.print("\t");Serial.print(digitalRead(ON_OFF));Serial.print("\t");Serial.print(PrLow);Serial.print("\t");Serial.print(PrHigh);Serial.print("\t");Serial.print("time :");Serial.print("\t");Serial.print(timt,5);Serial.print("\t");Serial.println(delta,8);
        }   
        }
    }
 
 } 
 
    //PSet=Pr;
    if (abs(PSet-PSet_old)>0.5){
    psetI = PSet;  
   // WriteLCD();
    PSet_old=PSet;
    }
    
    Read_3Sensors(35); // read the pr sensor
    Input=avgPr50GP;
   if (togglestate == LOW) 
    {
      PSet=0;
      }   
    myPID.Compute();
    if((setting==0)||((setting==0)&&(ON_OFFstate==LOW)) )
    {
      Output=0;
      write4LCD();
      }

    dac.setVoltage(Output, false);
    //Serial.print(PSet); Serial.print("\t");  Serial.print(avgPr50GP);Serial.println("\t");
  //  Serial.print(millis()-t1);Serial.println("\t");// COMMENTED BY RAVINDER
    delay(15);
    
}

//------------------------------------------------------------------------------------------------------------
//--------------------   User Defined Function   -------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------
void calculate_offset(int N){
  double sv50=0; 
  //int N=200; //no for samplese to calculate offser
  for (int i=0;  i<N;i++){
    sv50=sv50+analogRead(PS_50GP);
    delay(5);// delay for 1 ms
    Vos50=(sv50/i)*adc2V ;
   //Serial.print("Calibrating "); Serial.println(Vos50);
   }
}


void Read_3Sensors(int Navg){
  float SV50 = analogRead(PS_50GP)*adc2V;
  float vout50=SV50-Vos50;
  float Pr50GP=vout50*113.3; //Vref*0.018=0.09; 0.018 is slope of voltage to pr graph (Kpa)
  avgPr50GP=(avgPr50GP*(Navg-1)+Pr50GP)/Navg; // Calculating Running Avrage (Kpa)
}
void read_pressure()
{
  float SV50 = analogRead(PS_50GP)*adc2V;
  float vout50=SV50-Vos50;
  float Pr50GP=vout50*113.3; //Vref*0.018=0.09; 0.018 is slope of voltage to pr graph (Kpa)
  //avgPr50GP=(avgPr50GP*(Navg-1)+Pr50GP)/Navg; // Calculating Running Avrage (Kpa)
   avgPr50GP = (1-alpha)* avgPr50GP + alpha*Pr50GP;
}




void Read_pot1(int Navg){
  Pot1Value = analogRead(Pot1Pin);// Read Pot1
  setPrAvg=(setPrAvg*(Navg-1)+Pot1Value)/Navg;//Calculating Running Avrage 

}
/*void callBreathRate()
{
  p_uptrend=uptrend;
  if (avgPr50GP>Priv_avgPr50GP)
  uptrend=true;
  else
  uptrend=false;
  if (p_uptrend!=uptrend)

}*/

 /* togglestate= digitalRead(toggle);
   delay(2500);
    if (togglestate == LOW) 
         {
        Pot1Value = analogRead(Pot1Pin);
        PrLow = map(Pot1Value, 0, 1023, 0, 30);
         lcdPrLow=PrLow;
    } */
void WriteLCD()
{
  lcd.clear();
  lcd.setCursor(0,0);
  //lcd.print("PrSet:"); lcd.print(psetI);
  lcd.print("Set Pr:"); lcd.print(lcdPrHigh); // Ravinder
  //lcd.setCursor(9,0); 
  //lcd.print("LP: "); lcd.print(lcdPrLow);
  lcd.setCursor(0,1);
 // lcd.print("RP:");
  //ON_OFFstate= digitalRead(ON_OFF); 
//  if(ON_OFFstate==HIGH)
//  {
//  lcd.print("OFF");
//  }
//  if(ON_OFFstate==LOW)
//  {
//  lcd.print("ON");
//  }
//  lcd.setCursor(9,1);
  lcd.print("Current Pr:"); lcd.print(pactI); //PrC is current pressure
  //lcd.print("PrHigh"); lcd.print(PrHigh); // ravinder
}
void write2LCD()
{ 
 // delay(); 
  lcd.clear();
  lcd.setCursor(0,0);
  //lcd.print("PrSet:"); lcd.print(psetI);
  lcd.print("HP:"); lcd.print(lcdPrHigh); // Ravinder
  lcd.setCursor(6,0); 
  lcd.print("T:"); lcd.print(timt);
  lcd.setCursor(11,0); 
  lcd.print("CP:"); lcd.print(pactI);
  lcd.setCursor(0,1);
  lcd.print("LP:");lcd.print(lcdPrLow);
  lcd.setCursor(6,1);
  lcd.print("D:"); lcd.print(duration);
  lcd.setCursor(13,1);
  //ON_OFFstate= digitalRead(ON_OFF); 
  if(ON_OFFstate==HIGH)
  {
  lcd.print(" ");
  }
  if(ON_OFFstate==LOW)
  {
  lcd.print("R");
  }
  //PrC is current pressure
  //lcd.print("PrHigh"); lcd.print(PrHigh); // ravinder
  
  }
  
  void write3LCD()
{  
  lcd.clear();lcd.setCursor(0,0);
  //lcd.print("PrSet:"); lcd.print(psetI);
  lcd.print("HP:"); lcd.print(lcdPrHigh); // Ravinder
  lcd.setCursor(6,0); 
  lcd.print("T:"); lcd.print(timt);
  lcd.setCursor(11,0); 
  lcd.print("CP:"); lcd.print(pactI);
  lcd.setCursor(0,1);
  lcd.print("LP:");lcd.print(PrLow);
  lcd.setCursor(6,1);
  lcd.print("D:"); lcd.print(settime);
  lcd.setCursor(13,1);
  //ON_OFFstate= digitalRead(ON_OFF); 
  if(ON_OFFstate==HIGH)
  {
  lcd.print(" ");
  }
  if(ON_OFFstate==LOW)
  {
  lcd.print("R");
  }
  //PrC is current pressure
  //lcd.print("PrHigh"); lcd.print(PrHigh); // ravinder
  
  }
  
void write4LCD()
{  
 // lcd.clear();lcd.setCursor(0,0);
  //lcd.print("PrSet:"); lcd.print(psetI);
  lcd.print("  CPAP:STANDBY"); 
  lcd.setCursor(0,1);
  lcd.print("PRESS SET TO RUN"); 
  }

 void flow_reading()
 {
  Wire.beginTransmission(byte(0x40)); // transmit to device #064 (0x40)
  Wire.write(byte(0x10));      //
  Wire.write(byte(0x00));      //
  Wire.endTransmission();
  Wire.requestFrom(0x40, 3); // read 3 bytes from device with address 0x40
  while (Wire.available()) { // slave may send less than requested
  int a = Wire.read(); // first received byte stored here. The variable "uint16_t" can hold 2 bytes, this will be relevant later//   uint16_t
  uint8_t b = Wire.read(); // second received byte stored here
  uint8_t crc = Wire.read(); // crc value stored here
  a = (a << 8) | b; // combine the two received bytes to a 16bit integer value
  // a >>= 2; // remove the two least significant bits
  flow = (a - offset) / scale;
  filtered_flow = (1-alpha)*filtered_flow + alpha*flow;

  //Serial.print(temp);
  //Serial.print("  ");
  //Serial.print(T1);
  //Serial.print("  "); 
 //  Serial.print(t5);
 //  Serial.print("  "); 
  Serial.print(IPAP);
  Serial.print("  "); 
  Serial.print(EPAP);
  Serial.print("  "); 
  Serial.print(avgPr50GP);
  Serial.print("  "); 
 // Serial.print(filtered_flow/10);
//  Serial.print("  "); 
  if(ppf_detected==true)
  {
   Serial.print(positive_peak_flow/10); 
   }
  if(npf_detected==true)
  {
   Serial.print(negative_peak_flow/10); 
   }
  Serial.print("  ");
  Serial.println(filtered_flow/10);  
  }
  }
  void peak_flow_detection()
  {
    if(peak_first>10) // to skip initial raw data
    {
    if(filtered_flow > previous_flow)
    {
      positive_peak_flow = filtered_flow;
      previous_flow= filtered_flow;
      npf_detected=true;
      ppf_detected =false;
    
     }
     if(filtered_flow < previous_flow)
     {
      negative_peak_flow = filtered_flow;
      previous_flow= filtered_flow;
      npf_detected=false;
      ppf_detected = true;
      }
    }
    else
    {
      peak_first = peak_first + 1;
      positive_peak_flow=0;
      }
    
     //Serial.println(peak_flow); 
    }
   void BIPAP()
   {
        
        Pot1Value = analogRead(Pot1Pin);
        IPAP = map(Pot1Value, 0, 1023, 4, 20);

        Pot2Value = analogRead(Pot2Pin);
        BPM = map(Pot2Value, 0, 1023, 0, 60);

        Pot3Value = analogRead(Pot3Pin);
        EPAP = map(Pot3Value, 0, 1023, 4, 20);

        IPAP=10;
        EPAP=5;
        BPM=20;
        Itime= ((60*0.25)/BPM);   // in seconds
        Etime= (60/BPM)-Itime;
        Itrigger= 0.8 * EPAP;   // 5% less of EPAP
        TrigPr= Itrigger;


       // Bipap operation starts from here

       PSet=EPAP;
       module_1();
       delay(1500); 


 // Bipap operation mainly starts from here
      
       label0: PSet=EPAP;
       t1=millis(); 
       Initial_T1=millis();
       T1=0;


       label1: module_1();

       if(avgPr50GP < TrigPr)
       {
        write6LCD();
        T1=millis()-Initial_T1;
        goto label2;
        }
        else
        { 
          T1=millis()-Initial_T1;
          if(T1 > (Etime*1000)) // Etime is in seconds
          {
            // display : no breath detected
            write5LCD();
            goto label2;
            }
          else
          {
            delay(10);
            goto label1;
            }
         }

         label2: PSet=IPAP; 
         Initial_T2=millis();
         T2=0;
         //ppf_detected = false;
         goto label3;

         label3: module_1();
         T2=millis()-Initial_T2;
         if(T2 > (Itime*1000)) // Itime is in seconds
         {
          // "display: Itime set too low"
          // COMPUTE BRatetemp=0;
          temp=(millis()-Initial_T1 - T1);
         // Serial.println(temp);
          BRate = (60000/temp);  // IN bpm per minute
          // display BRate
          
          write7LCD();
          delay(10);
           t4=millis();
          t5=t4-t1;
          //Serial.println(t5); 
          goto label0;
          }
          else
          {
            if(ppf_detected==true)
            {
              Etrigger = 0.8*positive_peak_flow; // 20% less than positive peak flow
              if(filtered_flow < Etrigger)
              {
                
                // COMPUTE BRate
               temp=(millis()-Initial_T1 - T1);
               //Serial.println(temp);
               BRate = (60000/temp); // In bpm, per minute
               
               // display BRate
               write8LCD();
               delay(10);
                t4=millis();
                t5=t4-t1;
                //Serial.println(t5); 
               goto label0;
                }
              else
              {
                delay(10);
                goto label3;
                
                }
              
              }
              else
              {
                delay(10);
                goto label3;
                }
            
            }

     
    }
    void module_1()
    { 
       //Read_3Sensors(35);
       read_pressure();
       Input=avgPr50GP;
       myPID.Compute();
       dac.setVoltage(Output, false);
       flow_reading();
       peak_flow_detection();
      
      }
void write5LCD()
{  
 lcd.clear();lcd.setCursor(0,0);
 
  lcd.print("NO BREATH DETECT"); 
  lcd.setCursor(0,1);
  lcd.print("PRESS SET TO RUN"); 
  }
void write6LCD()
{  
 lcd.clear();lcd.setCursor(0,0);
 
  lcd.print("BREATH DETECTED"); 
  lcd.setCursor(0,1);
  lcd.print("PRESS SET TO RUN"); 
  }
  void write7LCD()
{  
 lcd.clear();lcd.setCursor(0,0);
 
  lcd.print("Itime set low"); 
  lcd.setCursor(0,1);
  lcd.print(BRate); 
  }
void write8LCD()
{  
 lcd.clear();lcd.setCursor(0,0);
 
  lcd.print("Itime set HIGH"); 
  lcd.setCursor(0,1);
  lcd.print(BRate); 
  }
    
