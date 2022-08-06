#include <math.h>
#include <SD.h>
#include <Wire.h> //Needed for I2C to BMW280
#include <MsTimer2.h>
#include <SparkFunBME280.h>
#include <avr/wdt.h>
#include <avr/io.h>
#include <Servo.h>
#include <mcp_can.h>
#include <EEPROM.h>

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
#define SW1 10
#define CAN0_INT 40                   // Set INT to pin 40
////////////////////

////Timer3のDuty設定用レジスタ名////
#define O2PWMset OCR3A  
#define AirPWMset OCR3B
#define LPGPWMset OCR3C
///////////////////////////////////

////SDカードファイル名////
#define FILE_NAME "DataLog"

//////////グローバル変数の宣言//////////

//各制御の目標値
float r_a = REF_AIR;  //L/min
float r_g = REF_GAS;  //L/min
float Kp_a = KP_AIR;//空気のPゲイン
float Ki_a = KI_AIR;//空気のIゲイン
const float Kd_a = 0;//空気のDゲイン
float Kp_g = KP_GAS;//LPGのPゲイン
float Ki_g = KI_GAS;//LPGのIゲイン
const float Kd_g = 0;//LPGのDゲイン
int OffSet_a = OFFSET_AIR;//PWMのオフセット
int OffSet_g = OFFSET_GAS;
//流量系統の積分偏差の上限下限設定
const int sum_max =  5;
const int sum_min = -5;

float Temp = 0.0;
float Humidity = 0.0;
float Pressure = 0.0;
#ifdef BME_OUT_EN
float Temp_out = 0.0;
float Humidity_out = 0.0;
float Pressure_out = 0.0;
#endif
float Flow_data[3]={0};
uint16_t PWM_data[3]={0};
int16_t timecount=0, IG_count=0 ;
int16_t Tc_val_LowPass=0 , Tc_val=0 , Tc_diff[TC_BUFF_NUM]={0} , Tc_diff_sum=0;
uint8_t IG_flag=0 , Flow_flag=0 , time_flag=0 , IG_point[4]={0} , Pulse_Count = 0 , Flow_delay=0 , SD_flag=0 ,LPG_EN=1 , LPG_delay=0 , NNN=0 , Tc_buff_num=0 ,Light_flag=0 ;
double etmp_a = 0, sum_a = 0;
double etmp_g = 0, sum_g = 0;
String Buffer_BME280;
#ifdef BME_OUT_EN
String Buffer_BME280_OUT;
#endif
String Buffer_Flow;
String FileName = FILE_NAME;
String RECEVE_Str_LoRa;
////////////////////////////////////

////////EEPROMのアドレス設定//////////
#define EEP_ADRS0 0 //r_a
#define EEP_ADRS1 EEP_ADRS0 + sizeof(r_a) //r_g
#define EEP_ADRS2 EEP_ADRS1 + sizeof(r_g) //Ki_a
#define EEP_ADRS3 EEP_ADRS2 + sizeof(Ki_a) //Ki_g
#define EEP_ADRS4 EEP_ADRS3 + sizeof(Ki_g) //OffSet_a
#define EEP_ADRS5 EEP_ADRS4 + sizeof(OffSet_a) //OffSet_g
////////////////////////////////////

/////////////クラスの宣言/////////////
BME280 bme280;
#ifdef BME_OUT_EN
BME280 bme280_out;
#endif
File myFile;
Servo Servo_Diaphragm;
MCP_CAN CAN0(42);  // Set CS to pin 42
///////////////////////////////////

////////////関数の宣言//////////////
void pinSetup(void);
void setupBME280(void);
void BME280_data(void);
void SDsetup(void);
void SDWriteData(void);
void CANsetup(void);
void Air_Control();
void LPG_Control();
void IG_Get_LoRa();
void EEPROM_Load();
void IG_Pulse();
void IG_heater();
void REIG();
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
  pinMode(SW1,INPUT);
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
    Serial2.println("NO SD");
    //while (1);  //ウォッチドッグに引っかかって再起動する
  }
  else Serial2.println("SD OK");
  myFile = SD.open(FileName+".csv", FILE_WRITE);
  myFile.write("Time(ms),");
  myFile.write("Temp_IN,Humidity_IN,Pressure_IN,Temp_Out,Humidity_Out,Pressure_Out,");
  myFile.write("Air_y,LPG_y,Air_u,LPG_u");
  myFile.println(",Thermo");
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
//    myFile.write(',');
    #ifdef BME_OUT_EN
    myFile.print(Buffer_BME280_OUT);
    myFile.write(',');
    #endif
    myFile.print(Buffer_Flow);
    
  }
}
//////////////////////////////////////////////////////////////////////

///////////////////////////////CAN////////////////////////////////////
void CANsetup(){
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK){
    Serial.println("MCP2515 Initialized Successfully!");
    CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.
  }
  else
    Serial.println("Error Initializing MCP2515...");
  
}

void CAN_read(){
  long unsigned int rxId;
  unsigned char len = 0;
  unsigned char rxBuf[8];
  if(!digitalRead(CAN0_INT)) {                // If CAN0_INT pin is low, read receive buffer
    CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
    if(rxId == 0x200){  //気圧による点火30秒ver
      REIG();
      time_flag=1;
      Serial2.println("PrIG");
    }
    else if(rxId == 0x300){//気圧による点火ずっとver
      REIG();
      Serial2.println("IG");
    }
  }
  
}


//////////////////////////////////////////////////////////////////////

///////////////////////////BME280////////////////////////////////
void setupBME280(void) {
  bme280.setI2CAddress(0x76);
  if (bme280.beginI2C()) {
    Wire.beginTransmission(0x76);
    Wire.write(0xf4);
    Wire.write(0x93);
    Wire.endTransmission();
    #ifdef DEBUG_SENS
    Serial.println("BME280 adress 0x76(in) OK");
    #endif
    Serial2.print("IN OK,");
    #ifndef BME_OUT_EN
    return;
    #endif
    delay(50);
  }
  bme280_out.setI2CAddress(0x77);
  if (bme280_out.beginI2C()) {
    Wire.beginTransmission(0x77);
    Wire.write(0xf4);
    Wire.write(0x93);
    Wire.endTransmission();
    #ifdef DEBUG_SENS
    Serial.println("BME280 adress 0x77(out) OK");
    #endif
    Serial2.print("OUT OK,");
    return;
  }
  #ifdef DEBUG_SENS
  Serial.println("Sensor connect failed");
  #endif
  Serial2.print("NO BME,");
}

void BME280_data(void) {
  Temp = bme280.readTempC(); //°C
  Humidity = bme280.readFloatHumidity(); //%
  Pressure = bme280.readFloatPressure() / 100; //hPa
  #ifdef BME_OUT_EN
  Temp_out = bme280_out.readTempC(); //°C
  Humidity_out = bme280_out.readFloatHumidity(); //%
  Pressure_out = bme280_out.readFloatPressure() / 100; //hPa
  #endif
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

void Create_Buffer_BME280_OUT(void){
  Buffer_BME280_OUT.remove(0);
  Buffer_BME280_OUT.concat(Temp_out);
  Buffer_BME280_OUT.concat(","); 
  Buffer_BME280_OUT.concat(Humidity_out);
  Buffer_BME280_OUT.concat(",");
  Buffer_BME280_OUT.concat(Pressure_out);
}

void Create_Buffer_Flow(){
  Buffer_Flow.remove(0);
  /*Buffer_Flow.concat(Flow_data[0]);
  Buffer_Flow.concat(","); */
  Buffer_Flow.concat(Flow_data[1]);
  Buffer_Flow.concat(",");
  Buffer_Flow.concat(Flow_data[2]);
  Buffer_Flow.concat(",");
  /*Buffer_Flow.concat(PWM_data[0]);
  Buffer_Flow.concat(","); */
  Buffer_Flow.concat(PWM_data[1]);
  Buffer_Flow.concat(",");
  Buffer_Flow.concat(PWM_data[2]);
}
//////////////////////////////////////////////////////////////////

////////////////////////LoRa通信/////////////////////////////

void IG_Get_LoRa(){
  char receve_tmp = 0;
  int16_t command_val=0 , eq_count=0 ;
  float command_val_f=0;
  String TMP_Str;
  
  while(Serial2.available()>0){
      receve_tmp = Serial2.read();
      if(receve_tmp == 0x08 ) RECEVE_Str_LoRa.remove(RECEVE_Str_LoRa.length()-1,1);
      else RECEVE_Str_LoRa.concat(receve_tmp);
  }
  if(receve_tmp == '\n'){
      if(RECEVE_Str_LoRa.equals("REIG\r\n") ){
        REIG();
        Serial2.println("Get REIG");
      }
      else if(RECEVE_Str_LoRa.equals("OK\r\n") );
      else if(RECEVE_Str_LoRa.equals("RESET\r\n") ){
        Serial2.println("Get RESET");
        digitalWrite(RST_mega,HIGH);
        delay(100);
        digitalWrite(RST_mega,LOW);
      }
      else if(RECEVE_Str_LoRa.equals("LOG\r\n") ){
          SD_flag = !SD_flag;
          Serial2.print("Log ");
          if(SD_flag) Serial2.print("start");
          else Serial2.print("stop");
          Serial2.println();
      }
      else if(RECEVE_Str_LoRa.equals("SENS\r\n") ){
        if(SD_flag == 0) BME280_data();
        Serial2.print(Pressure);
        Serial2.write(",");
        Serial2.print(Flow_data[1]);
        Serial2.write(",");
        Serial2.print(Flow_data[2]);
        Serial2.write(",");
        Serial2.println(Tc_val_LowPass);
      }
      else if(RECEVE_Str_LoRa.equals("BME\r\n") ){
        if(SD_flag == 0){
          BME280_data();
          Create_Buffer_BME280();
        }
        Serial2.println(Buffer_BME280);
      }
      else if(RECEVE_Str_LoRa.equals("BMEOUT\r\n") ){
        if(SD_flag == 0){
          BME280_data();
          Create_Buffer_BME280_OUT();
        }
        Serial2.println(Buffer_BME280_OUT);
      }
      else if(RECEVE_Str_LoRa.equals("FLOW\r\n") ){
        Serial2.println(Buffer_Flow);
      }
      else if(RECEVE_Str_LoRa.equals("STA\r\n") ){
         Serial2.print("SD=");
         Serial2.print(SD_flag);
         Serial2.print(",Fl=");
         Serial2.print(Flow_flag);
         /*Serial2.print(",L=");
         Serial2.print(LPG_EN);*/
         Serial2.print(",LF=");
         Serial2.print(Light_flag);
         if(NNN != 0)Serial2.print(", EL");
         Serial2.println();
      }
      else if(RECEVE_Str_LoRa.equals("TIME\r\n") ){        
         Serial2.print((float)millis()/60000);
         Serial2.println("min");
      }
      else if(RECEVE_Str_LoRa.equals("LPGCUT\r\n") ){
        LPG_EN = !LPG_EN;
        if(LPG_EN) Serial2.println("EN");
        else Serial2.println("Cut");
      }
      else if(RECEVE_Str_LoRa.equals("EARTHLIGHT\r\n") ){
         NNN = !NNN;
         Serial2.println("Get");
      }
      else if (RECEVE_Str_LoRa.startsWith("MKFL:")){
        if(RECEVE_Str_LoRa.length()>24 || RECEVE_Str_LoRa.length()<8) Serial2.println("Err");
        else {
           FileName = RECEVE_Str_LoRa.substring(5);
           FileName.remove(FileName.length()-2,2);
           myFile.close();
           myFile = SD.open(FileName+".csv", FILE_WRITE);
           myFile.write("Time(ms),");
           myFile.write("Temp_IN,Humidity_IN,Pressure_IN,Temp_Out,Humidity_Out,Pressure_Out,");
           myFile.write("Air_y,LPG_y,Air_u,LPG_u");
           myFile.println(",Thermo");
           myFile.flush();
           Serial2.println(FileName);
        }                 
      }
      else if(RECEVE_Str_LoRa.charAt(0) == '$' ){
         if(RECEVE_Str_LoRa.startsWith("$r_g")){
            eq_count = RECEVE_Str_LoRa.indexOf('=');
            if(eq_count != -1){
              RECEVE_Str_LoRa.remove(0,eq_count+1);
              command_val_f = RECEVE_Str_LoRa.toFloat();
              if(command_val_f>0.2 || command_val_f<0.01) Serial2.println("Err");
              else r_g = command_val_f;
            }            
            Serial2.print("r_g=");
            Serial2.println(r_g);
          }
          else if(RECEVE_Str_LoRa.startsWith("$r_a")){
            eq_count = RECEVE_Str_LoRa.indexOf('=');
            if(eq_count != -1){
              RECEVE_Str_LoRa.remove(0,eq_count+1);
              command_val_f = RECEVE_Str_LoRa.toFloat();
              if(command_val_f>0.99 || command_val_f<0.01) Serial2.println("Err");
              else r_a = command_val_f;
            }            
            Serial2.print("r_a=");
            Serial2.println(r_a);
          }
           else if(RECEVE_Str_LoRa.startsWith("$Kp_g")){
             if(sscanf(RECEVE_Str_LoRa.c_str() , "$Kp_g=%d\r\n" ,&command_val)){
               if(command_val>10000 || command_val<0) Serial2.println("Err");
               else Kp_g = (float)command_val;
             }
             Serial2.print("Kp_g=");
             Serial2.println(Kp_g);             
          }
          else if(RECEVE_Str_LoRa.startsWith("$Kp_a") ){
             if(sscanf(RECEVE_Str_LoRa.c_str() , "$Kp_a=%d\r\n" ,&command_val)){
               if(command_val>10000 || command_val<0) Serial2.println("Err");
               else Kp_a = (float)command_val;
             }
             Serial2.print("Kp_a=");
             Serial2.println(Kp_a);
          }
           else if(RECEVE_Str_LoRa.startsWith("$Ki_g")){
             if(sscanf(RECEVE_Str_LoRa.c_str() , "$Ki_g=%d\r\n" ,&command_val)){
               if(command_val>10000 || command_val<0) Serial2.println("Err");
               else Ki_g = (float)command_val;
             }
             Serial2.print("Ki_g=");
             Serial2.println(Ki_g);             
          }
          else if(RECEVE_Str_LoRa.startsWith("$Ki_a")){
             if(sscanf(RECEVE_Str_LoRa.c_str() , "$Ki_a=%d\r\n" ,&command_val)){
               if(command_val>10000 || command_val<0) Serial2.println("Err");
               else Ki_a = (float)command_val;
             }
             Serial2.print("Ki_a=");
             Serial2.println(Ki_a);
          }
          else if(RECEVE_Str_LoRa.startsWith("$OffSet_g")){
            if(sscanf(RECEVE_Str_LoRa.c_str() , "$OffSet_g=%d\r\n" ,&command_val)){
              if(command_val>4095 || command_val<0) Serial2.println("Err");
              else OffSet_g = command_val;
            }
            Serial2.print("OffSet_g=");
            Serial2.println(OffSet_g);
          }
          else if(RECEVE_Str_LoRa.startsWith("$OffSet_a")){
            if(sscanf(RECEVE_Str_LoRa.c_str() , "$OffSet_a=%d\r\n" ,&command_val)){
              if(command_val>4095 || command_val<0) Serial2.println("Err");
              else OffSet_a = command_val;
            }
            Serial2.print("OffSet_a=");
            Serial2.println(OffSet_a);
          }
          else if(RECEVE_Str_LoRa.equals("$LOAD\r\n")  ){
            EEPROM_Load();
            Serial2.println("LOAD");
          }
          else if(RECEVE_Str_LoRa.equals("$SAVE\r\n")  ){
            EEPROM.put(EEP_ADRS0,r_a);
            EEPROM.put(EEP_ADRS1,r_g);
            EEPROM.put(EEP_ADRS2,Ki_a);
            EEPROM.put(EEP_ADRS3,Ki_g);
            EEPROM.put(EEP_ADRS4,OffSet_a);
            EEPROM.put(EEP_ADRS5,OffSet_g);
            Serial2.println("SAVE");
          }
          else if(RECEVE_Str_LoRa.equals("$DEFAULT\r\n")  ){
            r_a = REF_AIR;  //L/min
            r_g = REF_GAS; //L/min
            Kp_a = KP_AIR;
            Ki_a = KI_AIR;
            Kp_g = KP_GAS; 
            Ki_g = KI_GAS;
            OffSet_a = OFFSET_AIR;
            OffSet_g = OFFSET_GAS;
            Serial2.println("Set DEFAULT");
          }
          else Serial2.println("?");
      }
      else Serial2.println("?");
      RECEVE_Str_LoRa.remove(0);
  }
}

void EEPROM_Load(){   //EEPROMから設定読み出し
    EEPROM.get(EEP_ADRS0,r_a);
    EEPROM.get(EEP_ADRS1,r_g);
    EEPROM.get(EEP_ADRS2,Ki_a);
    EEPROM.get(EEP_ADRS3,Ki_g);
    EEPROM.get(EEP_ADRS4,OffSet_a);
    EEPROM.get(EEP_ADRS5,OffSet_g);
}

void REIG(){    //点火消火
  IG_flag = 1;
  Flow_delay = 0;
  IG_count = HEATER_TIME;
  if(Flow_flag==1) {  //消火動作の場合
    Flow_flag=0;
    Flow_delay = 1;
    LPG_delay=0;
    if(PWM_data[1]<3000)  OffSet_a = PWM_data[1];   //空気のオフセットを保存
    if(PWM_data[2]<2000)  OffSet_g = PWM_data[2];   //LPGのオフセットを保存
  }  
}

void IG_heater(){
  if(IG_flag != 0){     //点火シーケンス中か
    if(IG_count > 0){   //ニクロム線の通電時間中か
      analogWrite(IGPWM,255);     //ニクロム線に通電
      if(Flow_delay == 0 ){   //点火シーケンス中にLPGを遅れて流すための条件判定
        Flow_flag=1;
        if( IG_count == (int16_t)HEATER_TIME/2){    //ニクロム線通電時間の半分からLPG噴射開始
          LPG_delay=1;
        }
      }
      IG_count--;
    }
    else {
      IG_flag = 0;  //点火シーケンス終了
      analogWrite(IGPWM,0);
    }
  }
  else if(NNN != 0) analogWrite(IGPWM,240);
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
