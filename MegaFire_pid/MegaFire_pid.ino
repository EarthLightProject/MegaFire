#define DEBUG_FLOW  //流量系統のデバッグ
#define DEBUG_SENS  //センサ系のデバッグ用
#define BME_OUT_EN //外側BMEを有効化

#define Serial2 Serial //LoRaからのコマンドをシリアルモニタから使う
//////////制御定数定義/////////////
//各制御の目標値
#define REF_AIR 0.7  //L/min
#define REF_GAS 0.08  //L/min

//空気のPIゲイン
#define KP_AIR 0
#define KI_AIR 2700

//LPGのPIゲイン
#define KP_GAS 0
#define KI_GAS 3000

//PWMのオフセット
#define OFFSET_AIR 1950
#define OFFSET_GAS 1600

////////////////////////////////////

//////////////時間定数//////////////
#define Ts 50 //(ms)タイマ割り込みの周期, 制御周期
#define SENDTIME 4  //送信間隔(s)
#define FLOW_TIME 20
#define HEATER_TIME 100  //ニクロムの導通時間　半分の時間から燃料を流す
#define Press_IG_TIME 30 //気圧による予備点火の時間(s)
#define LOW_PASS_COEFF 0.99   //熱電対のローパスフィルタ係数(0<x<1)
#define TC_BUFF_NUM 20        //熱電対の値の微分の移動平均サンプル数
#define LIGHT_THRESHOLD 300    //点火判定の閾値
////////////////////////////////////

#include "MegaFire_pid.h"  //ライブラリとピン定義

void setup(){
  wdt_enable(WDTO_4S);   //4秒周期のウォッチドッグタイマ開始
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
  Serial2.println("MegaFire");
  //while(Serial2.read() != '\n');
  Serial.println("LoRa is Ready");
  delay(2000);
  wdt_reset();
  Wire.begin();          //I2C通信開始
  setupBME280();
  CANsetup();
  SDsetup();
  EEPROM_Load();
  Servo_Diaphragm.attach(Servo_PWM);
  Servo_Diaphragm.write(78);
  MsTimer2::set(Ts, TIME_Interrupt); // TsごとにTIME_Interruptを呼び出す
  MsTimer2::start();
}

void loop(){
  if(digitalRead(SW1)==1){    //SDカードのログ保存ボタン検知
    while(digitalRead(SW1)==1);
    SD_flag = !SD_flag;
    Serial.print("Log ");
    Serial2.print("Log ");
    if(SD_flag){
      Serial.print("start");
      Serial2.print("start");
    }
    else {
      Serial.print("stop");
      Serial2.print("stop");
    }
    Serial.println();
    Serial2.println();
  }
  if(SD_flag == 1){     //SDカードへログを保存
    BME280_data();
    Create_Buffer_BME280();
    #ifdef BME_OUT_EN
    Create_Buffer_BME280_OUT();
    #endif
    SDWriteData();
    myFile.write(',');
    myFile.print(Tc_val);
    myFile.println();
    myFile.flush(); 
  }
  CAN_read();       //CANからのメッセージを確認
  IG_Get_LoRa();    //LoRaからのメッセージを確認
  if(digitalRead(SWITCH)==0){
    while(digitalRead(SWITCH)==0);
    REIG();
  }
}

///////////////////////サブ関数////////////////////////////
void TIME_Interrupt(void){
  wdt_reset();
  
  if(time_flag == 1)   timecount++;
  if(timecount>(int16_t)(Press_IG_TIME*1000/Ts)){
    time_flag=0;
    timecount=0;
    REIG();
  }
  
  Tc_val = analogRead(Thermocouple_PIN);//熱電対の温度測定
  Tc_val_LowPass = (int16_t)(Tc_val_LowPass*LOW_PASS_COEFF + Tc_val*(1-LOW_PASS_COEFF));  //熱電対にローパスフィルタをかける
  Tc_diff_sum += Tc_val_LowPass - Tc_diff[Tc_buff_num]; //合計値に最新の値を足して一番古い値を引く
  Tc_diff[Tc_buff_num] = Tc_val_LowPass;                //バッファー内部も更新
  if((int)(Tc_diff_sum / TC_BUFF_NUM) > LIGHT_THRESHOLD) Light_flag = 1;    //点火判定 1で点火状態
  else if(-(int)(Tc_diff_sum / TC_BUFF_NUM) > LIGHT_THRESHOLD) Light_flag = 0;
  if(Tc_buff_num > (TC_BUFF_NUM - 2) )  Tc_buff_num = 0;
  else Tc_buff_num ++ ;
  
  if(Flow_flag==1){
    Air_Control();
    if(LPG_EN==1 && LPG_delay==1) LPG_Control();
    else{
      LPGPWMset=0;
      Serial.println();
    }
  }
  else {
    AirPWMset=0;
    LPGPWMset=0;
    sum_a = 0;
    sum_g = 0;
    Flow_data[1] = 0;
    Flow_data[2] = 0;
    PWM_data[1]=0;
    PWM_data[2]=0;
  }
  Create_Buffer_Flow(); //流量を保存
  IG_heater();  //

}

///////////////////////////////////////////////////////////

//////////////////////PID制御関数///////////////////////////

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
  if(sum_a > sum_max) sum_a = sum_max;
  else if(sum_a < sum_min) sum_a = sum_min;
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
  if(sum_g > sum_max) sum_g = sum_max;
  else if(sum_g < sum_min) sum_g = sum_min;
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
