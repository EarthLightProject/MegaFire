#include <math.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h> //Needed for I2C to BMW280
#include <MsTimer2.h>
#include <SparkFunBME280.h>
#include <avr/wdt.h>
#include <avr/io.h>
#include <Servo.h>

//pin define
#define GNSS_RST 22
#define Servo_PWM 46
#define IGsig 40
#define CS 53
#define IGPWM 11
#define O2_flow A0
#define Air_flow A1
#define LPG_flow A2
#define LPG_PWM 3
#define Air_PWM 2
#define O2_PWM 5
#define Thermocouple_PIN A4
#define LoRa_RESET 24
#define RST_mega 7


#define O2PWMset OCR3A  //Timer3のDuty設定用レジスタ名
#define AirPWMset OCR3B
#define LPGPWMset OCR3C

#define FILE_NAME "DataLog.txt"

//////////グローバル変数の宣言//////////
float Temp_IN=0.0,Temp_OUT=0.0;
float Humidity_IN=0.0,Humidity_OUT=0.0;
float Pressure_IN=0.0,Pressure_OUT=0.0;
float Flow_data_LoRa[3]={0};
int timecount=0, IG_count=0;
uint8_t IG_flag=0 , Flow_flag=0 , time_flag=0 , IG_point[4]={0} , Pulse_Count = 0 , delay_count=0;
String Buffer_BME280_OUT;
String Buffer_BME280_IN;
String RECEVE_Str;
////////////////////////////////////

/////////////クラスの宣言/////////////
BME280 BME280_OUT;
BME280 BME280_IN;
File myFile;
Servo Servo_Diaphragm;
///////////////////////////////////


////////////関数の宣言//////////////
void pinSetup(void);
void init_BME280(void);
void setupBME280(void);
void BME280_data(void);
void SDsetup(void);
void Serial_print(void);
void Create_Buffer(void);
void SDWriteData(void);
void Diaphragm_control(void);
void O2_Conrol();
void Air_Control();
void LPG_Control();
void IG_Get();
void Pressure_IG();
void IG_Pulse();
/////////////////////////////////

/****************************************************
  　　　　　　　プロトタイプ宣言した関数
 ****************************************************/
void pinSetup(){
  pinMode(IGsig, INPUT);
  pinMode(Servo_PWM,OUTPUT);
  pinMode(O2_PWM,OUTPUT);
  pinMode(LPG_PWM,OUTPUT);
  pinMode(Air_PWM,OUTPUT);
  pinMode(IGPWM,OUTPUT);
  pinMode(O2_flow,INPUT);
  pinMode(Air_flow,INPUT);
  pinMode(LPG_flow,INPUT);
  pinMode(Thermocouple_PIN,INPUT);
  pinMode(LoRa_RESET,OUTPUT);
  pinMode(RST_mega,OUTPUT);
  digitalWrite(LoRa_RESET,HIGH);
  digitalWrite(RST_mega,LOW);
  //pinMode(GNSS_RST,OUTPUT);
  //digitalWrite(GNSS_RST,HIGH);
}

////////////////////////////////SD Card//////////////////////////////
void SDsetup(){
  #ifdef DEBUG_SENS
  Serial.print("Initializing SD card...");
  #endif
  if (!SD.begin(CS)) {
    Serial.println("initialization failed!");
    //while (1);  //ウォッチドッグに引っかかって再起動する
  }
  myFile = SD.open(FILE_NAME, FILE_WRITE);
  myFile.write("Time(ms),");
  myFile.write("Temp_Out,Humidity_Out,PressureOut,Temp_In,Humidity_In,Pressure_In,");
  myFile.write("Latitude,Longitude,Height,");
  myFile.println("Hour,Min,Sec");
  myFile.flush(); 
  #ifdef DEBUG_SENS
  Serial.println("initialization done.");
  #endif
}

void SDWriteData(void) {
  if (myFile) {               // if the file opened okay, write to it:
    myFile.print(millis());
    myFile.write(',');
    myFile.print(Buffer_BME280_OUT);
    myFile.print(Buffer_BME280_IN);
  }
}
//////////////////////////////////////////////////////////////////////

///////////////////////////BME280////////////////////////////////
void setupBME280(void) {
  BME280_OUT.setI2CAddress(0x77);
  if (BME280_OUT.beginI2C()) {
    Wire.beginTransmission(0x77);
    Wire.write(0xf4);
    Wire.write(0x93);
    Wire.endTransmission();
    #ifdef DEBUG_SENS
    Serial.println("BME280 Outside OK");
    #endif
    //return;
    delay(50);
  }
  BME280_IN.setI2CAddress(0x76);
  if (BME280_IN.beginI2C()) {
    Wire.beginTransmission(0x76);
    Wire.write(0xf4);
    Wire.write(0x93);
    Wire.endTransmission();
    #ifdef DEBUG_SENS
    Serial.println("BME280 Inside OK");
    #endif
    return;
  }
  #ifdef DEBUG_SENS
  Serial.println("Sensor connect failed");
  #endif
}

void BME280_OUT_data(void) {
  Temp_OUT = BME280_OUT.readTempC(); //°C
  Humidity_OUT = BME280_OUT.readFloatHumidity(); //%
  Pressure_OUT = BME280_OUT.readFloatPressure() / 100; //hPa
}

void BME280_IN_data(void){
  Temp_IN = BME280_IN.readTempC(); //°C
  Humidity_IN = BME280_IN.readFloatHumidity(); //%
  Pressure_IN = BME280_IN.readFloatPressure() / 100; //hPa
}

//////////////////////////////////////////////////////////////////

///////////////////////////Buffer/////////////////////////////////
void Create_Buffer_BME280_OUT(void){
 // Buffer.concat("Latitude,Longitude,Height\n"); 
  Buffer_BME280_OUT.remove(0);
  Buffer_BME280_OUT.concat(Temp_OUT);
  Buffer_BME280_OUT.concat(","); 
  Buffer_BME280_OUT.concat(Humidity_OUT);
  Buffer_BME280_OUT.concat(",");
  Buffer_BME280_OUT.concat(Pressure_OUT);
  Buffer_BME280_OUT.concat(",");
}

void Create_Buffer_BME280_IN(void){
  Buffer_BME280_IN.remove(0);
  Buffer_BME280_IN.concat(Temp_IN);
  Buffer_BME280_IN.concat(","); 
  Buffer_BME280_IN.concat(Humidity_IN);
  Buffer_BME280_IN.concat(",");
  Buffer_BME280_IN.concat(Pressure_IN);
}

void IG_Get(int ig_time){
  if(Serial.available()>3){
     #ifdef DEBUG_SENS
       Serial.println("available");
     #endif
    do{
      RECEVE_Str.concat(char(Serial.read()));
    }while(Serial.available()>0);
    #ifdef DEBUG_SENS
    Serial.println(RECEVE_Str);
    #endif
    if(RECEVE_Str.compareTo("REIG") == 0){
       IG_flag = 1;
       Flow_flag = !Flow_flag;
       IG_count = ig_time;
       delay_count = IG_TIME_DELAY;
       #ifdef DEBUG_SENS
       Serial.println("get REIG");
       #endif
      }
      RECEVE_Str.remove(0);
    }
}

/*void Pressure_IG(){
  if(Pressure_OUT < 1.0) {
    
  }
  else if(Pressure_OUT < 490.0 && IG_point[0]==0){  //5km
    IG_point[0] = 1;
    IG_Pulse();
  }
  else if(Pressure_OUT < 240.0 && IG_point[1]==0){   //10km
     IG_point[1] = 1;
     IG_Pulse();
  }
  else if(Pressure_OUT < 115.0 && IG_point[2]==0){   //15km
     IG_point[2] = 1;
     IG_Pulse();
  }
  else if(Pressure_OUT < 57.0 && IG_point[3]==0){    //20km
     IG_point[3] = 1;
     IG_Pulse();
  }
}*/

void IG_Pulse(){
   IG_flag = 1;
   Flow_flag = 1;
   IG_count = IG_TIME+IG_TIME_DELAY;
   Pulse_Count = FLOW_TIME*20 ;
}

//////////////////////////////////////////////////////////////////

void change_freq1(int divide){    //PWMの設定をする関数 AVR.hで定義されているレジスタに直接設定を書き込む Timer1はイグナイタ、Timer3は流量制御に使用
  //TCCR1A = (TCCR1A & B00111100) | B10000010;   //Phase and frequency correct, Non-inverting mode, TOP defined by ICR
  TCCR3A = (TCCR3A & B00000000) | B10101010;   //Timer3のモード設定
  TCCR1B = (TCCR1B & 0b11111000) | divide;     //Timer1のプリスケーラ設定
  TCCR3B = (TCCR3B & B11100000) | B00010001;   //Timer3のプリスケーラ設定
  ICR3 = 4095;                                 //Timer3の分解能設定(4095=12bit)
}
