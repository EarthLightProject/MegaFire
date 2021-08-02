#define O2_flow A0
#define Air_flow A1
#define LPG_flow A2
#define LPG_PWM 2
#define Air_PWM 3
#define O2_PWM 5
#define IGPWM 11
#define O2PWMset OCR3A
#define AirPWMset OCR3C
#define LPGPWMset OCR3B
 
//#define DEBUG

//////////制御定数定義/////////////
//制御周期
#define Ts 50     //ms

//各制御の目標値
#define r_o 0.08  //L/min
#define r_a 0.7   //L/min
#define r_g 0.08  //L/min

//O2のPIDゲイン
#define Kp_o 0.01
#define Ki_o 0
#define Kd_o 0

//空気のPIDゲイン
#define Kp_a 0.01
#define Ki_a 0
#define Kd_a 0

//LPGのPIDゲイン
#define Kp_g 0.01
#define Ki_g 0
#define Kd_g 0

//PWMのオフセット
#define OffSet 2000
//流量系統の積分偏差の上限下限設定
#define sum_max  5
#define sum_min -5
/////////////////////////////////////

void change_freq1(int divide);
void O2_Conrol();
void Air_Control();
void LPG_Control();

void setup() {
  pinMode(O2_flow,INPUT);
  pinMode(Air_flow,INPUT);
  pinMode(LPG_flow,INPUT);
  pinMode(O2_PWM,OUTPUT);
  pinMode(LPG_PWM,OUTPUT);
  pinMode(Air_PWM,OUTPUT);
  pinMode(IGPWM,OUTPUT);
  change_freq1(2);//31.37kHz
  Serial.begin(9600);
}

void loop() {
  O2_Control();
  Air_Control();
  LPG_Control();
  Serial.println();
  delay(Ts);
}

void change_freq1(int divide){
  //TCCR1A = (TCCR1A & B00111100) | B10000010;   //Phase and frequency correct, Non-inverting mode, TOP defined by ICR
  TCCR3A = (TCCR3A & B00000000) | B10101010;   //Phase and frequency correct, Non-inverting mode, TOP defined by ICR
  TCCR1B = (TCCR1B & 0b11111000) | divide;
  TCCR3B = (TCCR3B & B11100000) | B00010001;   //No prescale
  ICR3 = 4095;
}

void O2_Control(){
  /* 変数設定 */
  static double etmp_o = 0, sum_o = 0; //1ステップ前の誤差, 誤差の総和
  double x = 0; //現在の流量
  int16_t u = 0; //制御入力
  x = analogRead(O2_flow);
  x = x * 5 / 1024;
  x = 0.0192 * x * x + 0.0074 * x - 0.0217; //流量の線形フィッティング
  /* 制御計算 */
  double e = r_o - x; //誤差
  u = int16_t(Kp_o * e + Ki_o * sum_o + Kd_o * (e - etmp_o) / (Ts * 1e-3) + OffSet); //制御入力を計算
  etmp_o = e; //1ステップ前の誤差を更新
  sum_o += (Ts * 1e-3) * e; //誤差の総和を更新
  /* 上下限設定 */
  if(sum_o > sum_max) sum_o = sum_max;
  else if(sum_o < sum_min) sum_o = sum_min;
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
  static double etmp_a = 0, sum_a = 0;
  double x = 0;
  int16_t u = 0;
  x = analogRead(Air_flow);
  x = x * 5 / 1024;
  x = 0.0528 * x * x -0.0729 * x + 0.0283;
  double e = r_a - x;
  /* 制御計算 */
  u = int16_t(Kp_a * e + Ki_a * sum_a + Kd_a * (e - etmp_a) / (Ts * 1e-3) + OffSet);
  etmp_a = e;
  sum_a += (Ts * 1e-3) * e;
  /* 上下限設定 */
  if(sum_a > sum_max) sum_a = sum_max;
  else if(sum_a < sum_min) sum_a = sum_min;
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
  static double etmp_g = 0, sum_g = 0;
  double x = 0;
  int16_t u = 0;
  x = analogRead(LPG_flow);
  x = x * 5 / 1024;
  x = 0.0528 * x * x -0.0729 * x+ 0.0283;
  double e = r_g - x;
  /* 制御計算 */
  u = int16_t(Kp_g * e + Ki_g * sum_g + Kd_g * (e - etmp_g) / (Ts * 1e-3) + OffSet);
  etmp_g = e;
  sum_g += (Ts * 1e-3) * e;
  /* 上下限設定 */
  if(sum_g > sum_max) sum_g=sum_max;
  else if (sum_g < sum_min) sum_g = sum_min;
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
