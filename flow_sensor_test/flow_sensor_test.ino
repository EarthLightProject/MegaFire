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

/////////Control Constant////////////
//各ガスの流量目標値
#define O2flow_Target 0.08  //L/min
#define Airflow_Target 0.7  //L/min
#define LPGflow_Target 0.08  //L/min

//O2のPID項
#define O2flow_P 400  //O2制御の比例項
#define O2flow_I 500
#define O2flow_D 200

//空気のPID項
#define Airflow_P 200  //空気制御の比例項
#define Airflow_I 300
#define Airflow_D 100

//LPGのPID項
#define LPGflow_P 400  //LPG制御の比例項
#define LPGflow_I 500
#define LPGflow_D 200

//PWMのオフセット
#define OffSet 2000
//積分偏差の上限下限設定
#define integral_MAX 5
#define integral_MIN -5
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
  delay(50);
}

void change_freq1(int divide){
  //TCCR1A = (TCCR1A & B00111100) | B10000010;   //Phase and frequency correct, Non-inverting mode, TOP defined by ICR
  TCCR3A = (TCCR3A & B00000000) | B10101010;   //Phase and frequency correct, Non-inverting mode, TOP defined by ICR
  TCCR1B = (TCCR1B & 0b11111000) | divide;
  TCCR3B = (TCCR3B & B11100000) | B00010001;   //No prescale
  ICR3 = 4095;
}

void O2_Control(){
  double flow_data = 0;
  int16_t Control = 0;
  static double integral = 0;
  static double difference=0;
  flow_data = analogRead(O2_flow);
  flow_data = flow_data*5/1024;
  flow_data = 0.0192*flow_data*flow_data + 0.0074*flow_data - 0.0217;
  Serial.print("O2=");
  Serial.print( flow_data);
  Serial.write(',');
  Control = int16_t(O2flow_P*(O2flow_Target - flow_data)+O2flow_I*integral+O2flow_D*(flow_data - difference)+OffSet);
  integral += O2flow_Target - flow_data;
  if(integral > integral_MAX ) integral=integral_MAX;
  else if (integral < integral_MIN) integral = integral_MIN;
  difference = flow_data;
  if(Control > 4095) Control = 4095;
  else if(Control < 0) Control = 0;
  O2PWMset = Control;//O2 PWM
  #ifdef DEBUG
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
  Serial.print("Air=");
  Serial.print( flow_data);
  Serial.write(',');
  Control = int16_t(Airflow_P*(Airflow_Target - flow_data)+Airflow_I*integral+Airflow_D*(flow_data - difference)+OffSet);
  integral += Airflow_Target - flow_data;
  if(integral > integral_MAX ) integral = integral_MAX;
  else if (integral < integral_MIN) integral = integral_MIN;
  difference = flow_data;
  if(Control > 4095) Control = 4095;
  else if(Control < 0) Control = 0;
  AirPWMset = Control;//Air PWM
  #ifdef DEBUG
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
  Serial.print("LPG=");
  Serial.print( flow_data);
  Control = int16_t(LPGflow_P*(LPGflow_Target - flow_data)+LPGflow_I*integral+LPGflow_D*(flow_data - difference)+OffSet);
  integral += LPGflow_Target - flow_data;
  if(integral > integral_MAX ) integral=integral_MAX;
  else if (integral < integral_MIN) integral = integral_MIN;
  difference = flow_data;
  if(Control > 4095) Control = 4095;
  else if(Control < 0) Control = 0;
  LPGPWMset = Control;//LPG PWM
  #ifdef DEBUG
  Serial.write(',');
  Serial.print(Control);
  #endif
}
