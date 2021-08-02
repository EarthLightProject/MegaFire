//#define DEBUG_PRESS   //気圧制御のデバッグ用
#define DEBUG_FLOW  //流量系統のデバッグ
//#define DEBUG_FLOW_LORA //LoRa越しの流量デバッグ
//#define DEBUG_SENS  //センサ系のデバッグ用

//////////制御定数定義/////////////
//各制御の目標値
#define O2flow_Target 0.06  //L/min
#define Airflow_Target 0.7  //L/min
#define LPGflow_Target 0.05  //L/min
#define Press_Target  1013.25; //気圧目標値hPa

//O2のPID項
#define O2flow_P 300  //O2制御の比例項
#define O2flow_I 500
#define O2flow_D 100

//空気のPID項
#define Airflow_P 300  //空気制御の比例項
#define Airflow_I 270
#define Airflow_D 120

//LPGのPID項
#define LPGflow_P 400  //LPG制御の比例項
#define LPGflow_I 500
#define LPGflow_D 130

//燃焼器内気圧のPID項
#define Param_P 0.05
#define Param_I 0.03
#define Param_D 0.01

//流量系統PWMのオフセット
#define OffSet 2000
#define OffSet_Diaphragm (30)

//流量系統の積分偏差の上限下限設定
#define integral_MAX  5
#define integral_MIN -5

//ダイアフラム制御の積分偏差の上限下限
#define integ_Diaphragm_MAX 30
#define integ_Diaphragm_MIN -3000

#define Servo_INPUT_MAX 105
#define Servo_INPUT_MIN 37
#define Servo_INVERT 127

//ダイアフラム制御周期
#define D_COUNT 1 //×POLLING(ms)
/////////////////////////////////////

//////////////通信系定数//////////////
#define IG_TIME 30 //イグナイタ点火時間
#define IG_TIME_DELAY 50 //イグナイタの点火遅れ時間(先に燃料噴射)
#define POLLING 50 //(ms)タイマ割り込みの周期デフォ50
#define SENDTIME 4  //送信間隔(s)デフォ4
#define FLOW_TIME 20
/////////////////////////////////////

#include "integration_for_Arduino_Mega4.h"  //ライブラリとピン定義


float Servo_Input=0;

void setup()
{
  pinSetup();            //IOピンの設定
  change_freq1(2);       //PWMの周期変更31.37kHz
  wdt_enable(WDTO_4S);   //8秒周期のウォッチドッグタイマ開始
  analogWrite(IGPWM,0);
  Serial.begin(9600);    //LoRaとの通信開始
  Serial.println("Hello");
  GNSSsetup();
  wdt_reset();
  Wire.begin();          //I2C通信開始
  setupBME280();
  SDsetup();
  Servo_Diaphragm.attach(Servo_PWM);
  MsTimer2::set(POLLING, TIME_Interrupt); // POLLINGごとにオンオフ
  MsTimer2::start();
  
}

void loop()
{
  BME280_OUT_data();
  BME280_IN_data();
  Create_Buffer_BME280_OUT();
  Create_Buffer_BME280_IN();
  SDWriteData();
  IG_Get(IG_TIME+IG_TIME_DELAY); 
  if(time_flag!=0){
      GNSS_data();
      Create_Buffer_GNSS();
      Create_Buffer_TIME();
      myFile.write(',');
      myFile.print(Buffer_GNSS);
      myFile.write(',');
      myFile.print(Buffer_TIME);
      time_flag=0;
      if(timecount > (int)(SENDTIME*1000/POLLING)){
        Serial_print();
        RECEVE_Str.remove(0);
        /*if((IG_flag != 1)|| (Pressure_OUT<310.0&&Pressure_OUT>1.0&&IG_count<1)){
          Serial.write(",IG");
          myFile.write(",IG");
        }*/
        Serial.write(',');
        if(analogRead(Thermocouple_PIN)>250&&Flow_flag==1)  Serial.write('2');
        else Serial.print(Flow_flag);
        Serial.println(); 
        timecount=0;
      }
    }
  Pressure_IG();
  myFile.println();
  myFile.flush(); 
}

///////////////////////サブ関数////////////////////////////
void TIME_Interrupt(void){
  static uint8_t Diaphram_count=0;
  wdt_reset();
  timecount++;
  if(timecount>(int)(1000/POLLING)) time_flag=1;
 /* if(Diaphram_count>D_COUNT){
    sei();
    BME280_OUT_data();
    BME280_IN_data();
    //Pressure_IN = BME280_IN.readFloatPressure() / 100; //hPa
    cli();
    Diaphragm_control();
    Diaphram_count=0;
  }
  else Diaphram_count++;*/
  
  if(Flow_flag==1){
    O2_Control();
    Air_Control();
    LPG_Control();
  }
  else {
    O2PWMset=0;
    AirPWMset=0;
    LPGPWMset=0;
  }
  
  if(IG_flag==1){
    IG_count--;
    if(IG_count<1){
      IG_flag=0;
      analogWrite(IGPWM,0);
      }
    else if(IG_count<(IG_TIME_DELAY)) analogWrite(IGPWM,30);
    }
   if(Pulse_Count>0){
    Pulse_Count--;
    if(Pulse_Count<2) Flow_flag=0;
   }
}

void Serial_print(void){
 Serial.print(Buffer_BME280_OUT);
 //Serial.write(',');
 #ifdef DEBUG_FLOW_LORA
 Serial.print(Flow_data_LoRa[0]);
 Serial.write(',');
 Serial.print(Flow_data_LoRa[1]);
 Serial.write(',');
 Serial.print(Flow_data_LoRa[2]);
 #else
 Serial.print(Buffer_GNSS); 
 #endif
}
///////////////////////////////////////////////////////////

//////////////////////PID制御関数///////////////////////////
void Diaphragm_control(){
  static float last_deviation=0 , Integral_deviation=0;
  float deviation_P = Press_Target;
  
  deviation_P -= Pressure_IN ;
  Servo_Input = OffSet_Diaphragm - (Param_P*deviation_P + Param_I*Integral_deviation + Param_D*(deviation_P - last_deviation));
  
  #ifdef DEBUG_PRESS
  +Serial.print(Pressure_IN);
  Serial.write(',');
  Serial.println(Servo_Input);
  #endif
  
  if(Servo_Input > Servo_INPUT_MAX) Servo_Input=Servo_INPUT_MAX;
  else if(Servo_Input<Servo_INPUT_MIN) Servo_Input=Servo_INPUT_MIN;
  Servo_Diaphragm.write(Servo_Input);
  last_deviation = deviation_P;
  Integral_deviation += deviation_P;
  if(Integral_deviation > integ_Diaphragm_MAX) Integral_deviation = integ_Diaphragm_MAX;
  else if(Integral_deviation < integ_Diaphragm_MIN) Integral_deviation = integ_Diaphragm_MIN;

}

void O2_Control(){
  double flow_data = 0;
  int16_t Control = 0;
  static double integral = 0;
  static double difference=0;
  flow_data = analogRead(O2_flow);
  flow_data = flow_data*5/1024;
  flow_data = 0.0192*flow_data*flow_data + 0.0074*flow_data - 0.0217;
  Flow_data_LoRa[0] = flow_data;
  Control = int16_t(O2flow_P*(O2flow_Target - flow_data)+O2flow_I*integral+O2flow_D*(flow_data - difference)+OffSet);
  integral += O2flow_Target - flow_data;
  if(integral > integral_MAX ) integral=integral_MAX;
  else if (integral < integral_MIN) integral = integral_MIN;
  difference = flow_data;
  if(Control > 4095) Control = 4095;
  else if(Control < 0) Control = 0;
  O2PWMset = Control;//O2 PWM
  #ifdef DEBUG_FLOW
  Serial.print("O2=");
  Serial.print( flow_data);
  Serial.write(',');
  Serial.print(Control);
  Serial.write(',');
  #endif
}

void Air_Control(){
  double flow_data = 0;
  int16_t Control = 0;
  static double integral = 0;
  static double difference=0;
  flow_data = analogRead(Air_flow);
  flow_data = flow_data*5/1024;
  flow_data =0.0528*flow_data*flow_data -0.0729*flow_data+ 0.0283;
  Flow_data_LoRa[1] = flow_data;
  Control = int16_t(Airflow_P*(Airflow_Target - flow_data)+Airflow_I*integral+Airflow_D*(flow_data - difference)+OffSet);
  integral += Airflow_Target - flow_data;
  if(integral > integral_MAX ) integral = integral_MAX;
  else if (integral < integral_MIN) integral = integral_MIN;
  difference = flow_data;
  if(Control > 4095) Control = 4095;
  else if(Control < 0) Control = 0;
  AirPWMset = Control;//Air PWM
  #ifdef DEBUG_FLOW
  Serial.print("Air=");
  Serial.print( flow_data);
  Serial.write(',');
  Serial.print(Control);
  Serial.write(',');
  #endif
}

void LPG_Control(){
  double flow_data = 0;
  int16_t Control = 0;
  static double integral = 0;
  static double difference=0;
  flow_data = analogRead(LPG_flow);
  flow_data = flow_data*5/1024;
  flow_data =0.0528*flow_data*flow_data -0.0729*flow_data+ 0.0283;
  Flow_data_LoRa[2] = flow_data;
  Control = int16_t(LPGflow_P*(LPGflow_Target - flow_data)+LPGflow_I*integral+LPGflow_D*(flow_data - difference)+OffSet);
  integral += LPGflow_Target - flow_data;
  if(integral > integral_MAX ) integral=integral_MAX;
  else if (integral < integral_MIN) integral = integral_MIN;
  difference = flow_data;
  if(Control > 4095) Control = 4095;
  else if(Control < 0) Control = 0;
  LPGPWMset = Control;//LPG PWM
  #ifdef DEBUG_FLOW
  Serial.print("LPG=");
  Serial.print( flow_data);
  Serial.write(',');
  Serial.println(Control);
  #endif
}
