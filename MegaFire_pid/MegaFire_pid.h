#include <math.h>
#include <SD.h>
#include <Wire.h> //Needed for I2C to BMW280
#include <MsTimer2.h>
#include <SparkFunBME280.h>
#include <avr/wdt.h>
#include <avr/io.h>
#ifdef DIAPHRAM_ENABLE
#include <Servo.h>
#endif

////ピン番号の定義////
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
#define SW1 8
#define SW2 9
#define SW3 10
////////////////////

////Timer3のDuty設定用レジスタ名////
#define O2PWMset OCR3A  
#define AirPWMset OCR3B
#define LPGPWMset OCR3C
///////////////////////////////////

////SDカードファイル名////
#define FILE_NAME "DataLog.txt"

//////////グローバル変数の宣言//////////
float Temp = 0.0;
float Humidity = 0.0;
float Pressure = 0.0;
float Flow_data[3]={0};
int timecount=0, IG_count=0;
uint8_t IG_repeat=0 , Flow_flag=0 , time_flag=0 , IG_point[4]={0} , Pulse_Count = 0 , delay_count=0 , SD_flag=0;
float etmp_d = 0 , sum_d = 0; //1ステップ前の誤差, 誤差の総和
double etmp_o = 0, sum_o = 0; //1ステップ前の誤差, 誤差の総和
double etmp_a = 0, sum_a = 0;
double etmp_g = 0, sum_g = 0;
String Buffer_BME280;
String Buffer_Flow;
String RECEVE_Str;
String RECEVE_Str_LoRa;
////////////////////////////////////

/////////////クラスの宣言/////////////
BME280 bme280;
File myFile;
#ifdef DIAPHRAM_ENABLE
Servo Servo_Diaphragm;
#endif
///////////////////////////////////

////////////関数の宣言//////////////
void pinSetup(void);
void init_BME280(void);
void setupBME280(void);
void BME280_data(void);
void SDsetup(void);
void Serial_print(void);
void SDWriteData(void);
void Diaphragm_control(void);
void O2_Conrol();
void Air_Control();
void LPG_Control();
void IG_Get();
void IG_Get2();
void IG_Pulse();
/////////////////////////////////

/****************************************************
  　　　　　　　プロトタイプ宣言した関数
 ****************************************************/
void pinSetup(){    //IOピンの設定
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
    myFile.print(Buffer_BME280);
    myFile.write(',');
    myFile.print(Buffer_Flow);
  }
}
//////////////////////////////////////////////////////////////////////

///////////////////////////BME280////////////////////////////////
void setupBME280(void) {
  bme280.setI2CAddress(0x77);
  if (bme280.beginI2C()) {
    Wire.beginTransmission(0x77);
    Wire.write(0xf4);
    Wire.write(0x93);
    Wire.endTransmission();
    #ifdef DEBUG_SENS
    Serial.println("BME280 adress 0x77 OK");
    #endif
    return;
    delay(50);
  }
  bme280.setI2CAddress(0x76);
  if (bme280.beginI2C()) {
    Wire.beginTransmission(0x76);
    Wire.write(0xf4);
    Wire.write(0x93);
    Wire.endTransmission();
    #ifdef DEBUG_SENS
    Serial.println("BME280 adress 0x76 OK");
    #endif
    return;
  }
  #ifdef DEBUG_SENS
  Serial.println("Sensor connect failed");
  #endif
}

void BME280_data(void) {
  Temp = bme280.readTempC(); //°C
  Humidity = bme280.readFloatHumidity(); //%
  Pressure = bme280.readFloatPressure() / 100; //hPa
}

//////////////////////////////////////////////////////////////////

///////////////////////////Buffer/////////////////////////////////
void Create_Buffer_BME280(void){
  Buffer_BME280.remove(0);
  Buffer_BME280.concat(Temp);
  Buffer_BME280.concat(","); 
  Buffer_BME280.concat(Humidity);
  Buffer_BME280.concat(",");
  Buffer_BME280.concat(Pressure);
  Buffer_BME280.concat(",");
}

void Create_Buffer_Flow(){
  Buffer_Flow.remove(0);
  Buffer_Flow.concat(Flow_data[0]);
  Buffer_Flow.concat(","); 
  Buffer_Flow.concat(Flow_data[1]);
  Buffer_Flow.concat(",");
  Buffer_Flow.concat(Flow_data[2]);
}
//////////////////////////////////////////////////////////////////

////////////////////////イグナイタ関係/////////////////////////////
void IG_Get(){
  char receve_tmp = 0;
  while(Serial.available()>0){
      receve_tmp = Serial.read();
      RECEVE_Str.concat(receve_tmp);
  }
  if(receve_tmp == '\n'){
      #ifdef DEBUG_SENS
      Serial.println("available");
      Serial.println(RECEVE_Str);
      #endif
      if(RECEVE_Str.compareTo("REIG\r\n") == 0){
        IG_repeat = IG_REPEAT;
        Flow_flag = !Flow_flag;
        IG_count = IG_TIME;
        delay_count = IG_TIME_DELAY;
        #ifdef DEBUG_SENS
        Serial.println("get REIG");
        #endif
      }
      else if(RECEVE_Str.compareTo("RESET\r\n") == 0){
        digitalWrite(RST_mega,HIGH);
      }
      RECEVE_Str.remove(0);
   }
}

void IG_Get_LoRa(){
  char receve_tmp = 0;
  while(Serial2.available()>0){
      receve_tmp = Serial2.read();
      RECEVE_Str_LoRa.concat(receve_tmp);
  }
  if(receve_tmp == '\n'){
      #ifdef DEBUG_SENS
      Serial.println("available");
      Serial.println(RECEVE_Str);
      #endif
      if(RECEVE_Str_LoRa.compareTo("REIG\r\n") == 0){
        IG_repeat = IG_REPEAT;
        Flow_flag = !Flow_flag;
        IG_count = IG_TIME;
        delay_count = IG_TIME_DELAY;
        #ifdef DEBUG_SENS
        Serial.println("get REIG");
        #endif
      }
      else if(RECEVE_Str_LoRa.compareTo("RESET\r\n") == 0){
        digitalWrite(RST_mega,HIGH);
      }
      RECEVE_Str_LoRa.remove(0);
  }
}

void IG_Pulse(){
  if(IG_repeat>0){
    if(delay_count <1){
      IG_count--;
      analogWrite(IGPWM,30);
    }
    else delay_count--;
    if(IG_count<1){
      IG_repeat--;
      IG_count = IG_TIME;
      delay_count = IG_TIME_DELAY;
      analogWrite(IGPWM,0);
      }
  }
  else analogWrite(IGPWM,0);
}
//////////////////////////////////////////////////////////////////

////////////////////////PWM関係の設定//////////////////////////////
void PWM_setup(int divide){    //PWMの設定をする関数 AVR.hで定義されているレジスタに直接設定を書き込む Timer1はイグナイタ、Timer3は流量制御に使用
  //TCCR1A = (TCCR1A & B00111100) | B10000010;   //Phase and frequency correct, Non-inverting mode, TOP defined by ICR
  TCCR3A = (TCCR3A & B00000000) | B10101010;   //Timer3のモード設定
  TCCR1B = (TCCR1B & 0b11111000) | divide;     //Timer1のプリスケーラ設定
  TCCR3B = (TCCR3B & B11100000) | B00010001;   //Timer3のプリスケーラ設定
  ICR3 = 4095;                                 //Timer3の分解能設定(4095=12bit)
}
//////////////////////////////////////////////////////////////////
