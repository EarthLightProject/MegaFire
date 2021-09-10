//#define DEBUG_PRESS   //気圧制御のデバッグ用
#define DEBUG_FLOW  //流量系統のデバッグ
// #define DEBUG_FLOW_LORA //LoRa越しの流量デバッグ
#define DEBUG_SENS  //センサ系のデバッグ用
//#define DIAPHRAM_ENABLE //ダイアフラム制御の可否
#define IG_HEATER //ニクロム線による点火

//////////制御定数定義/////////////
//各制御の目標値
#define r_o 0.08  //L/min
#define r_a 0.7  //L/min
#define r_g 0.08  //L/min
#define r_d 1013.25; //気圧目標値hPa

//O2のPIDゲイン
const float Kp_o = 1;
const float Ki_o = 3500;
const float Kd_o = 0;

//空気のPIDゲイン
const float Kp_a = 0;
const float Ki_a = 3000;
const float Kd_a = 0;

//LPGのPIDゲイン
const float Kp_g = 0;
const float Ki_g = 3000;
const float Kd_g = 0;

//PWMのオフセット
const int OffSet_o = 1700;
const int OffSet_a = 1950;
const int OffSet_g = 1700;

//燃焼器内気圧のPID項
#define Kp_d 0.05
#define Ki_d 0.03
#define Kd_d 0.01
#define OffSet_d (30)

//流量系統の積分偏差の上限下限設定
const int sum_max =  5;
const int sum_min = -5;

//ダイアフラム制御の積分偏差の上限下限
#define sum_d_max 30
#define sum_d_min -3000

#define u_d_max 105
#define u_d_min 37
#define Servo_INVERT 127
////////////////////////////////////

//////////////時間定数//////////////
#define IG_TIME 2 //イグナイタ点火時間
#define IG_TIME_DELAY 5 //イグナイタの点火遅れ時間(先に燃料噴射)
#define IG_REPEAT 10  //イグナイタ動作の繰り返し回数
#define Ts 50 //(ms)タイマ割り込みの周期, 制御周期
#define SENDTIME 4  //送信間隔(s)
#define FLOW_TIME 20
#define HEATER_TIME 80
////////////////////////////////////

#include "MegaFire_pid.h"  //ライブラリとピン定義

void setup(){
  wdt_enable(WDTO_4S);   //8秒周期のウォッチドッグタイマ開始
  pinSetup();            //IOピンの設定
  PWM_setup(2);          //PWMの周期変更31.37kHz
  analogWrite(IGPWM,0);
  Serial.begin(9600);    //PCとの通信開始
  Serial.println("MegaFire_pid start!");
  delay(2000);
  wdt_reset();
  delay(2000);
  wdt_reset();
  Serial2.begin(115200);  //LoRaとの通信開始
  Serial2.println("MegaFire start!");
  while(Serial2.read() != '\n');
  Serial.println("LoRa is Lady");
  Wire.begin();          //I2C通信開始
  setupBME280();
  SDsetup();
  Servo_Diaphragm.attach(Servo_PWM);
  Servo_Diaphragm.write(93);
  MsTimer2::set(Ts, TIME_Interrupt); // TsごとにTIME_Interruptを呼び出す
  MsTimer2::start();
}

void loop(){
  if(digitalRead(SW1)==1){
    while(digitalRead(SW1)==1);
    SD_flag = !SD_flag;
    Serial.print("Log ");
    if(SD_flag) Serial.print("start");
    else Serial.print("stop");
    Serial.println();
    Serial2.print("Log ");
    if(SD_flag) Serial2.print("start");
    else Serial2.print("stop");
    Serial2.println();
  }
  if(SD_flag==1){
    BME280_data();
    Create_Buffer_BME280();
    SDWriteData();
    myFile.println();
    myFile.flush(); 
  }
  IG_Get();
  IG_Get_LoRa();
}

///////////////////////サブ関数////////////////////////////
void TIME_Interrupt(void){
  wdt_reset();
  timecount++;
  if(timecount>(int)(1000/Ts)) time_flag=1;
  
  if(Flow_flag==1){
    O2_Control();
    Air_Control();
    LPG_Control();
    if(SD_flag == 1) Create_Buffer_Flow(); //ロギング中ならば流量を保存
  }
  else {
    O2PWMset=0;
    AirPWMset=0;
    LPGPWMset=0;
    sum_o = 0;
    sum_a = 0;
    sum_g = 0;
    Flow_data[0] = 0;
    Flow_data[1] = 0;
    Flow_data[2] = 0;
    PWM_data[0]=0;
    PWM_data[1]=0;
    PWM_data[2]=0;
  }
  #ifndef IG_HEATER
  IG_Pulse(); //イグナイタの動作
  #else
  IG_heater();
  #endif

  #ifdef DIAPHRAM_ENABLE
    sei();
    BME280_data();
    Diaphragm_control();
  #endif
}

void Serial_print(void){
  Serial.print(Buffer_BME280);
  #ifdef DEBUG_FLOW_LORA
  Serial.println(Buffer_Flow);
  #endif
}
///////////////////////////////////////////////////////////

//////////////////////PID制御関数///////////////////////////
#ifdef DIAPHRAM_ENABLE
void Diaphragm_control(){
  /* 変数設定 */
  float u_d = 0;
  float y_d = Pressure; //現在の圧力
  float e_d = r_d - y_d; //誤差
  /* 制御計算 */
  // u_d = OffSet_d - (Kp_d * e_d + Ki_d * sum_d + Kd_d * (e_d - etmp_d) / (Ts * 1e-3)); //制御入力を計算
  u_d = Kp_d * e_d + Ki_d * sum_d + Kd_d * (e_d - etmp_d) / (Ts * 1e-3); //制御入力を計算
  etmp_d = e_d; //誤差を更新
  sum_d += (Ts * 1e-3) * e_d; //誤差の総和を更新
  /* 上下限設定 */
  if(sum_d > sum_d_max) sum_d = sum_d_max;
  else if(sum_d < sum_d_min) sum_d = sum_d_min;
  if(u_d > u_d_max) u_d = u_d_max;
  else if(u_d < u_d_min) u_d = u_d_min;
  /* 入力 */
  Servo_Diaphragm.write(u_d);
  #ifdef DEBUG_PRESS
  Serial.print(y_d);
  Serial.write(',');
  Serial.println(u_d);
  #endif
}
#endif

void O2_Control(){
  /* 変数設定 */
  double y = 0; //現在の流量
  int16_t u = 0; //制御入力
  y = analogRead(O2_flow);
  y = y * 5 / 1024; //(V) 電圧値に戻す
  y = 0.0192 * y * y + 0.0074 * y - 0.0217; //流量の線形フィッティング
  double e = r_o - y; //誤差
  /* 制御計算 */
  u = int16_t(Kp_o * e + Ki_o * sum_o + Kd_o * (e - etmp_o) / (Ts * 1e-3) + OffSet_o); //制御入力を計算
  etmp_o = e; //1ステップ前の誤差を更新
  sum_o += (Ts * 1e-3) * e; //誤差の総和を更新
  /* 上下限設定 */
  if(u > 4095) u = 4095;
  else if(u < 0) u = 0;
  /* 入力 */
  O2PWMset = u; //O2 PWM
  Flow_data[0] = y;
  PWM_data[0] = u;
  #ifdef DEBUG_FLOW
  Serial.print("O2=");
  Serial.print(y);
  Serial.write(',');
  Serial.print(u);
  Serial.write(',');
  #endif
}

void Air_Control(){
  /* 変数設定 */
  double y = 0;
  int16_t u = 0;
  y = analogRead(Air_flow);
  y = y * 5 / 1024;
  y = 0.0528 * y * y -0.0729 * y + 0.0283;
  double e = r_a - y;
  /* 制御計算 */
  u = int16_t(Kp_a * e + Ki_a * sum_a + Kd_a * (e - etmp_a) / (Ts * 1e-3) + OffSet_a);
  etmp_a = e;
  sum_a += (Ts * 1e-3) * e;
  /* 上下限設定 */
  if(u > 4095) u = 4095;
  else if(u < 0) u = 0;
  /* 入力 */
  AirPWMset = u;//Air PWM
  Flow_data[1] = y;
  PWM_data[1] = u;
  #ifdef DEBUG_FLOW
  Serial.print("Air=");
  Serial.print(y);
  Serial.write(',');
  Serial.print(u);
  Serial.write(',');
  #endif
}

void LPG_Control(){
  /* 変数設定 */
  double y = 0;
  int16_t u = 0;
  y = analogRead(LPG_flow);
  y = y * 5 / 1024;
  y = 0.0192 * y * y + 0.0074 * y - 0.0217;
  double e = r_g - y;
  /* 制御計算 */
  u = int16_t(Kp_g * e + Ki_g * sum_g + Kd_g * (e - etmp_g) / (Ts * 1e-3) + OffSet_g);
  etmp_g = e;
  sum_g += (Ts * 1e-3) * e;
  /* 上下限設定 */
  if(u > 4095) u = 4095;
  else if(u < 0) u = 0;
  /* 入力 */
  LPGPWMset = u;//LPG PWM
  Flow_data[2] = y;
  PWM_data[2] = u;
  #ifdef DEBUG_FLOW
  Serial.print("LPG=");
  Serial.print(y);
  Serial.write(',');
  Serial.println(u);
  #endif
}
