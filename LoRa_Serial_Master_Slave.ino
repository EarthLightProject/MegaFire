#define DGHARDUINO_PIN_LORA_POWER (6)
#define DGHARDUINO_PIN_LORA_RESET (7)
#define DGHARDUINO_PIN_LORA_WAKEUP (8)
#define DGHARDUINO_PIN_CPU_LORA_UART_RX (9)
#define DGHARDUINO_PIN_CPU_LORA_UART_TX (10)
#define DGHARDUINO_PIN_CPU_LORA_SDA (A4)
#define DGHARDUINO_PIN_CPU_LORA_SCL (A5)

//#define IGTIME 5000

#define Master  //マスターでない場合コメントアウト
#define DEBUG

#ifndef Master
String AT_SEND = "AT+SENDP=1000:";
#else
String AT_SEND = "AT+SENDP=FFFF:";
#endif

String rfBuf;
uint16_t getData = 0;/* serial1 データ バイト*/
uint8_t data_flag=0;
uint8_t IGflag=0;
uint32_t IGcount=0;

void setup() {
  // opens serial port, sets data rate to 9600 bp
  Serial.begin(9600);
  Serial1.begin(9600);
  pinMode(DGHARDUINO_PIN_LORA_POWER, OUTPUT);
  pinMode(DGHARDUINO_PIN_LORA_RESET, OUTPUT);
  pinMode(2,INPUT);
  pinMode(3,OUTPUT);
  loraPower(true);/*電源ピンをハイに設定する。*/
  loraReset(true);/*リセットピンをハイに設定する。*/
  delay(4000);
  Serial1.println("AT+CHANNELS=P:9212:1");//周波数とデータレート
  delay(50);
  Serial1.println("AT+DADDRP=10:00");//アドレスの設定
  
}
void loop() {
if(Serial.available()>0){   //送信命令
    char C = Serial.read();/* PCからデータを受信する。*/
    if(C != '\r'){
      AT_SEND += C ;
    }
    else{
      AT_SEND.concat((char)175);
      //AT_SEND +='\r';
      Serial1.println(AT_SEND);
      //Serial.println(AT_SEND);
      AT_SEND.remove(14);
      //digitalWrite(3,LOW);
    }
    
 
 }
 
if (Serial1.available()){   //受信命令
    int inByte = Serial1.read();
    if((inByte >= 0x20) && (inByte <= 0x7f)){
      rfBuf.concat((char)inByte);
    }
    if((rfBuf.indexOf("PAYLOAD:", 0)>=0)&&(rfBuf.indexOf(",HOP:", 0)<0)){
      if((0x30<=inByte && inByte <= 0x39)||(0x41<=inByte && inByte <= 0x46)){
        if(getData<16){
          getData = 16*convertHex2Dec(inByte);
        }else{
          getData = getData + convertHex2Dec(inByte);
            if(getData==175){
              #ifndef Master
              Serial.println();//改行なしでMega側が受信
              #endif
              #ifdef Master
              if(data_flag>3)
              {  
                //digitalWrite(3,HIGH);
                //Serial.print("REIG");
                //IGflag=1;
                Serial1.print("AT+SENDP=FFFF:Comannd Accept");
                Serial1.write((char)175);
                Serial1.println();
                delay(800);
                digitalWrite(DGHARDUINO_PIN_LORA_RESET,0);
                delay(5);
                digitalWrite(DGHARDUINO_PIN_LORA_RESET,1);
               }
               data_flag=0;
              #endif
            }else{
              #ifdef DEBUG
              Serial.print((char)getData);
              #endif
              #ifdef Master
              if('R'==(char)getData)  data_flag++;
              if('E'==(char)getData)  data_flag++;
              if('I'==(char)getData)  data_flag++;
              if('G'==(char)getData)  data_flag++;
              #endif
            }
          getData = 0;
        }
      }
    }
    if((inByte == 0x0D) || (inByte == 0x0A)){
            //Serial.println(rfBuf);
            rfBuf.remove(0);
            //Serial.println(rfBuf);
        }
        
       
 }
/*#ifdef Master
 if(IGflag!=0){
  IGcount++;
  if(IGcount>IGTIME){
    IGflag=0;
    IGcount=0;
    digitalWrite(3,LOW);
  }
 }
#endif*/
 
 #ifndef Master
 if(digitalRead(2)==0){
  Serial1.print("AT+SENDP=1000:REIG");
  Serial1.write((char)175);
  Serial1.println();
  Serial.println("send");
  delay(1000);
 }
 #endif

}

/******************************************************************************/
/*  名前: convertHex2Dec()                                                     */
/*  内容：この関数は、hexadecimalからdecimal                                      */
/*       　表示します。　                                                        */
/******************************************************************************/
int convertHex2Dec(int data){
    if(data<65){
      data -= 48;
    }else{
      data -= 55;
    }
  return data;
}
/*****************************************************************************/
/*  名前: loraPower()                                                      　 */
/*  内容：電源ピンを設定する。                                                    */
/*****************************************************************************/
void loraPower(bool bOn){
  digitalWrite(DGHARDUINO_PIN_LORA_POWER,bOn);
}
/******************************************************************************/
/*  名前: loraReset()                                                          */
/*  内容：リセットピンを設定する。                                                  */
/******************************************************************************/
void loraReset(bool bOn){
  digitalWrite(DGHARDUINO_PIN_LORA_RESET,bOn);
}
