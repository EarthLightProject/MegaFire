//#define DEBUG_PRESS   //気圧制御のデバッグ用
//#define DEBUG_FLOW  //流量系統のデバッグ
//#define DEBUG_SENS  //センサ系のデバッグ用

#include "integration_for_Arduino_Mega4_dob.h"  //ライブラリとピン定義

//////////制御定数定義/////////////
//各制御の目標値
#define r_o 0.08  //L/min
#define r_a 0.7  //L/min
#define r_g 0.08  //L/min
#define r_d 1013.25; //気圧目標値hPa

//O2のPID項
#define Kp_o 0.01  //O2制御の比例項
#define Ki_o 0
#define Kd_o 0

//空気のPID項
#define Kp_a 0.01  //空気制御の比例項
#define Ki_a 0
#define Kd_a 0

//LPGのPID項
#define Kp_g 0.01  //LPG制御の比例項
#define Ki_g 0
#define Kd_g 0

//燃焼器内気圧のPID項
#define Kp_d 0.01
#define Ki_d 0
#define Kd_d 0

//流量系統PWMのオフセット
#define OffSet 2000
#define OffSet_Diaphragm (30)

//流量系統の積分偏差の上限下限設定
#define sum_max  5
#define sum_min -5

//ダイアフラム制御の積分偏差の上限下限
#define sum_d_max 30
#define sum_d_min -3000

#define u_d_max 105
#define u_d_min 37
#define Servo_INVERT 127

//ダイアフラム制御周期
#define D_COUNT 2 //×Ts(ms)
/////////////////////////////////////

//////////////通信系定数//////////////
#define IG_TIME 30 //イグナイタ点火時間
#define IG_TIME_DELAY 50
#define Ts 50 //(ms)タイマ割り込みの周期, 制御周期
#define SENDTIME 4  //送信間隔(s)
/////////////////////////////////////

float u_d = 0;

void setup()
{
  pinSetup();            //IOピンの設定
  change_freq1(2);       //PWMの周期変更31.37kHz
  wdt_enable(WDTO_4S);   //8秒周期のウォッチドッグタイマ開始
  Serial.begin(9600);    //LoRaとの通信開始
  Serial.println("Hello");
  GNSSsetup();
  Wire.begin();          //I2C通信開始
  setupBME280();
  Servo_Diaphragm.attach(Servo_PWM);
  MsTimer2::set(Ts, TIME_Interrupt); // TsごとTIME_Interruptを呼び出す
  MsTimer2::start();
}

void loop()
{
  BME280_OUT_data();
  BME280_IN_data();
  //Diaphragm_control();
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
      if(timecount > (int)(SENDTIME*1000/Ts)){
        Serial_print();
        RECEVE_Str.remove(0);
        /*if((digitalRead(IGsig)==1) && (IG_flag != 1)|| (Pressure_OUT<310.0&&Pressure_OUT>1.0&&IG_count<1)){
          Serial.write(",IG");
          myFile.write(",IG");
        }*/
        Serial.write(',');
        Serial.print(Flow_flag);
        Serial.println(); 
        timecount=0;
      }
    }
  myFile.println();
  myFile.flush(); 
}

///////////////////////サブ関数////////////////////////////
void TIME_Interrupt(void){
  static uint8_t Diaphram_count=0;
  wdt_reset();
  timecount++;
  if(timecount>(int)(1000/Ts)) time_flag=1;
  
  /*if(Diaphram_count>D_COUNT){
    sei();
    x_d = BME280_IN.readFloatPressure() / 100; //hPa
    //cli();
    Diaphragm_control();
    Diaphram_count=0;
  }*/
  else Diaphram_count++;
  
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
}

void Serial_print(void){
 Serial.print(Buffer_BME280_IN);
 Serial.print(Buffer_GNSS); 
}
///////////////////////////////////////////////////////////

//////////////////////PID制御関数///////////////////////////
void Diaphragm_control(){
  /* 変数設定 */
  static float e_d_tmp = 0 , sum_d = 0; //1ステップ前の誤差, 誤差の総和
  float x_d = Pressure_IN; //現在の圧力
  float e_d = r_d - x_d; //誤差
  /* 制御計算 */
  u_d = OffSet_Diaphragm - (Kp_d * e_d + Ki_d * sum_d + Kd_d * (e_d - e_d_tmp) / (Ts * 1e-3)); //制御入力を計算
  e_d_tmp = e_d; //誤差を更新
  sum_d += (Ts * 1e-3) * e_d; //誤差の総和を更新
  /* 上下限設定 */
  if(sum_d > sum_d_max) sum_d = sum_d_max;
  else if(sum_d < sum_d_min) sum_d = sum_d_min;
  if(u_d > u_d_max) u_d = u_d_max;
  else if(u_d < u_d_min) u_d = u_d_min;
  /* 入力 */
  Servo_Diaphragm.write(u_d);
  #ifdef DEBUG_PRESS
  +Serial.print(x_d);
  Serial.write(',');
  Serial.println(u_d);
  #endif
}

void O2_Control(){
  /* 変数設定 */
  static double e_o_tmp = 0, sum = 0; //1ステップ前の誤差, 誤差の総和
  double x = 0; //現在の流量
  int16_t u = 0; //制御入力
  x = analogRead(O2_flow);
  x = x * 5 / 1024;
  x = 0.0192 * x * x + 0.0074 * x - 0.0217; //流量の線形フィッティング
  /* 制御計算 */
  double e = r_o - x; //誤差
  u = int16_t(Kp_o * e + Ki_o * sum + Kd_o * (e - e_o_tmp) / (Ts * 1e-3) + OffSet); //制御入力を計算
  e_o_tmp = e; //1ステップ前の誤差を更新
  sum += (Ts * 1e-3) * e; //誤差の総和を更新
  /* 上下限設定 */
  if(sum > sum_max) sum = sum_max;
  else if (sum < sum_min) sum = sum_min;
  if(u > 4095) u = 4095;
  else if(u < 0) u = 0;
  /* 入力 */
  O2PWMset = u; //O2 PWM
  #ifdef DEBUG_FLOW
  Serial.print("O2=");
  Serial.print(x);
  Serial.write(',');
  Serial.print(u);
  Serial.write(',');
  #endif
}

void Air_Control(){
  /* 変数設定 */
  double x = 0;
  int16_t u = 0;
  static double sum = 0;
  static double e_a_tmp = 0;
  x = analogRead(Air_flow);
  x = x * 5 / 1024;
  x = 0.0528 * x * x -0.0729 * x + 0.0283;
  double e = r_a - x;
  /* 制御計算 */
  u = int16_t(Kp_a * e + Ki_a * sum + Kd_a * (e - e_a_tmp) / (Ts * 1e-3) + OffSet);
  e_a_tmp = e;
  sum += (Ts * 1e-3) * e;
  /* 上下限設定 */
  if(sum > sum_max) sum = sum_max;
  else if (sum < sum_min) sum = sum_min;
  if(u > 4095) u = 4095;
  else if(u < 0) u = 0;
  /* 入力 */
  AirPWMset = u;//Air PWM
  #ifdef DEBUG_FLOW
  Serial.print("Air=");
  Serial.print(x);
  Serial.write(',');
  Serial.print(u);
  Serial.write(',');
  #endif
}

void LPG_Control(){
  /* 変数設定 */
  double x = 0;
  int16_t u = 0;
  static double sum = 0;
  static double e_g_tmp = 0;
  x = analogRead(LPG_flow);
  x = x * 5 / 1024;
  x = 0.0528 * x * x -0.0729 * x+ 0.0283;
  double e = r_g - x;
  /* 制御計算 */
  u = int16_t(Kp_g * e + Ki_g * sum + Kd_g * (e - e_g_tmp) / (Ts * 1e-3) + OffSet);
  e_g_tmp = e;
  sum += (Ts * 1e-3) * e;
  /* 上下限設定 */
  if(sum > sum_max) sum=sum_max;
  else if (sum < sum_min) sum = sum_min;
  if(u > 4095) u = 4095;
  else if(u < 0) u = 0;
  /* 入力 */
  LPGPWMset = u;//LPG PWM
  #ifdef DEBUG_FLOW
  Serial.print("LPG=");
  Serial.print(x);
  Serial.write(',');
  Serial.println(u);
  #endif
}
