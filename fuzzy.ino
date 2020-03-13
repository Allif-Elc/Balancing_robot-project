#include "stdlib.h"
#include "math.h"
  float PWM;
  float settingPoint;
  float Kecepatan;
  float Adc;
  float Error, DeltaError, ErrL=0;
  float OutFuzzy;
  static float Integral=0;
  const int AnalogoutPin=A1;

  String dataIn;         // a string to hold incoming data
  String data[10];
  int i, X;
  boolean stringComplete = false;  // whether the string is complete

float dan (float a1, float a2)
{
  return ((a1<a2)?a1:a2);
}

float atau (float a1, float a2)
{
  return ((a1>a2)?a1:a2);
}

float MF(float i, float a, float b, float c)
{
  if(i<a||i>c) return 0;
  else if(i<b) return (i-a)/(b-a);
  else return (c-i)/(c-b);
}

float Fuzzy(float e)
{
  /*
  Program Kontroler Logika Fuzzy, EruP(c)2011-2015
  */
  static float LastError=0;
  float de;                  // delta error
  float ePB, ePM, ePS, eZ, eNS, eNM, eNB;
  float dPB, dPM, dPS, dZ, dNS, dNM, dNB;
  float oNH, oNB, oNM, oNS, oZ, oPS, oPM, oPB, oPH;
  float Out;
  
  
  #define pePB 100      // point Error Positive Big
  #define pePM 50       // point Error Positive Medium
  #define pePS 10       // point Error Positive Small
  #define peZ  0        // point Error Zero
  #define peNS -10      // point Error Negative Small
  #define peNM -50      // point Error Negative Medium
  #define peNB -100     // point Error Negative Big
  
  #define pdPB 1500     // point Delta Error Positive Big
  #define pdPM 1000     // point Delta Error Positive Medium
  #define pdPS 500      // point Delta Error Positive Small
  #define pdZ  0        // point Delta Error Zero
  #define pdNS -500     // point Delta Error Negative Small
  #define pdNM -1000    // point Delta Error Negative Medium
  #define pdNB -1500    // point Delta Error Negative Big
  
  #define poPH 255      // point of Output Positive Huge tone
  #define poPB 150      // point of Output Positive Big tone
  #define poPM 100      // point of Output Positive Medium tone
  #define poPS 50       // point of Output Positive Small tone
  #define poZ 0         // point of Output Zero tone
  #define poNS -50      // point of Output Negative Small tone
  #define poNM -100     // point of Output Negative Medium tone
  #define poNB -150     // point of Output Negative Big tone
  #define poNH -255     // point of Output Negative Huge tone

  de = e - LastError;
  LastError = e;
  
  // calculate fuzzy value using fuzzyfication based on membership function
  // calculate membership function for error signal and delta error  
  ePB=MF(e,pePM,pePB,1e38);  // Fungsi keanggotaan Positive Big untuk Error
  ePM=MF(e,pePS,pePM,pePB);  // Fungsi keanggotaan Positive Medium untuk Error
  ePS=MF(e,peZ,pePS,pePM);   // Fungsi keanggotaan Positive Small untuk Error
  eZ =MF(e,peNS,peZ,pePS);   // Fungsi keanggotaan Zero untuk Error
  eNS=MF(e,peNM,peNS,peZ);   // Fungsi keanggotaan Negative Small untuk Error
  eNM=MF(e,peNB,peNM,peNS);  // Fungsi keanggotaan Negative Medium untuk Error
  eNB=MF(e,-1e38,peNB,peNM); // Fungsi keanggotaan Negative Big untuk Error
  
  dPB=MF(de,pdPM,pdPB,1e38); // Fungsi keanggotaan Positive Big untuk Delta Error
  dPM=MF(de,pdPS,pdPM,pdPB); // Fungsi keanggotaan Positive Medium untuk Delta Error
  dPS=MF(de,pdZ,pdPS,pdPM);  // Fungsi keanggotaan Positive Small untuk Delta Error
  dZ =MF(de,pdNS,pdZ,pdPS);  // Fungsi keanggotaan Zero untuk Delta Error
  dNS=MF(de,pdNM,pdNS,pdZ);  // Fungsi keanggotaan Negative Small untuk Delta Error
  dNM=MF(de,pdNB,pdNM,pdNS); // Fungsi keanggotaan Negative Medium untuk Delta Error
  dNB=MF(de,-1e38,pdNB,pdNM); // Fungsi keanggotaan Negative Big untuk Delta Error
 
  // do inference engine using IF-THEN rule base
  
  oPH = atau(dan(ePB,dPS),atau(dan(ePM,dPM),atau(dan(ePS,dPB),atau(dan(ePB,dPM),atau(dan(ePM,dPB),dan(ePB,dPB))))));
  oPB = atau(dan(ePB,dZ),atau(dan(ePM,dPS),atau(dan(ePS,dPM),dan(eZ,dPB))));
  oPM = atau(dan(ePB,dNS),atau(dan(ePM,dZ),atau(dan(ePS,dPS),atau(dan(eZ,dPM),dan(eNS,dPB)))));
  oPS = atau(dan(ePB,dNM),atau(dan(ePM,dNS),atau(dan(ePS,dZ),atau(dan(eZ,dPS),atau(dan(eNS,dPM),dan(eNM,dPB))))));
  oZ  = atau(dan(ePB,dNB),atau(dan(ePM,dNM),atau(dan(ePS,dNS),atau(dan(eZ,dZ),atau(dan(eNS,dPS),atau(dan(eNM,dPM),dan(eNB,dPB)))))));
  oNS = atau(dan(ePM,dNB),atau(dan(ePS,dNM),atau(dan(eZ,dNS),atau(dan(eNS,dZ),atau(dan(eNM,dPS),dan(eNB,dPM))))));
  oNM = atau(dan(ePS,dNB),atau(dan(eZ,dNM),atau(dan(eNS,dNS),atau(dan(eNM,dZ),dan(eNB,dPB)))));
  oNB = atau(dan(eZ,dNB),atau(dan(eNS,dNM),atau(dan(eNM,dNS),(dan(eNB,dZ)))));
  oNH = atau(dan(eNS,dNB),atau(dan(eNM,dNM),atau(dan(eNB,dNS),atau(dan(eNM,dNB),atau(dan(eNB,dNM),dan(eNB,dNB))))));
  
  // calculate crisp value using defuzzyfication
  Out=(oNH*poNH+oNB*poNB+oNM*poNM+oNS*poNS+oZ*poZ+oPS*poPS+oPM*poPM+oPB*poPB+oPH*poPH)/(oNH+oNB+oNM+oNS+oZ+oPS+oPM+oPB+oPH);
  return Out;
}  
void setup() {
  // put your setup code here, to run once:
  
  // initialize serial:
  Serial.begin(19200);

  settingPoint=0;
  Integral=0;
  
}

void loop() {
    
    Adc=analogRead(A0);
    Kecepatan=Adc*2000/1023;
    Error=settingPoint-Kecepatan;
    DeltaError=Error-ErrL;
    ErrL=Error;

    OutFuzzy=Fuzzy(Error);
    Integral+=Error;
    PWM=OutFuzzy;
    if(PWM>255)PWM=255;
    if(PWM<0)PWM=0;
    analogWrite(AnalogoutPin,PWM);

    Serial.println(settingPoint);
    Serial.print('\t');
    Serial.println(Kecepatan);
    Serial.print('\t');
    Serial.println(PWM);
    Serial.print('\t');
    Serial.println(Error);
    Serial.print('\t');
    Serial.println(DeltaError);
    delay(20);
}
void serialEvent() {
  if (Serial.available() > 0) //jika ada serial
  {
    char inChar = (char)Serial.read();                //serial in char per char
    dataIn += inChar;                                 //tampung char

    if (inChar == ' ')                  //indikasi paket data selesai (CR)
    {
      stringComplete = true;
    }
  }
  if (stringComplete)
  {
    parsingData();                       //panggil fungsi parsing
    stringComplete = false;                     //parsing selesai
    dataIn = " ";                        //data penampung dihapus
  }
}

void parsingData()
{
  for (i=0;i<dataIn.length();i++){
    if(dataIn[i] == 's'){     //jika pengenal s untuk data
      X++;    // indeks penampung data+1
      data[X]=" ";  //kosongkan lagi sebelum diisi
    }
    else{
      data[X] = data[X] + dataIn[i]; //array indeks ke X penapung data dibuffer
    }
  }
  settingPoint = data[X].toFloat();
}


