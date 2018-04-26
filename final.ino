
#include <AlertNodeLib.h>   // to use XBee serial communications
#include <SoftwareSerial.h>
#include <IRremote.h>
#include "configuration.h"
int incomingByte;
const int txPin = 2;      // for XBee TX
const int rxPin = 3;      // for XBee RX
const String myNodeName = "Casa Andrea";
const boolean RECEIVING = false;
int isForward = 0;
AlertNode myNode;

 IRrecv IR(IR_PIN);  //   IRrecv object  IR get code from IR remoter
 decode_results IRresults;

/***************motor control***************/
void go_Advance(void)  //Forward
{
  isForward = 1;
  digitalWrite(dir1PinL, HIGH);
  digitalWrite(dir2PinL,LOW);
  digitalWrite(dir1PinR,HIGH);
  digitalWrite(dir2PinR,LOW);
  analogWrite(speedPinL,100);
  analogWrite(speedPinR,100);
  isForward = 0;
}
void go_Left(int t=0)  //Turn left
{
  digitalWrite(dir1PinL, HIGH);
  digitalWrite(dir2PinL,LOW);
  digitalWrite(dir1PinR,LOW);
  digitalWrite(dir2PinR,HIGH);
  analogWrite(speedPinL,200);
  analogWrite(speedPinR,200);
  delay(t);
}

void go_Left_Alert(int t=0)  //Turn left
{
  digitalWrite(dir1PinL, HIGH);
  digitalWrite(dir2PinL,LOW);
  digitalWrite(dir1PinR,LOW);
  digitalWrite(dir2PinR,HIGH);
  analogWrite(speedPinL,600);
  analogWrite(speedPinR,600);
  delay(t);
}

void go_Right(int t=0)  //Turn right
{
  digitalWrite(dir1PinL, LOW);
  digitalWrite(dir2PinL,HIGH);
  digitalWrite(dir1PinR,HIGH);
  digitalWrite(dir2PinR,LOW);
  analogWrite(speedPinL,200);
  analogWrite(speedPinR,200);
  delay(t);
}
void go_Right_Alert(int t=0)  //Turn right
{
  digitalWrite(dir1PinL, LOW);
  digitalWrite(dir2PinL,HIGH);
  digitalWrite(dir1PinR,HIGH);
  digitalWrite(dir2PinR,LOW);
  analogWrite(speedPinL,600);
  analogWrite(speedPinR,600);
  delay(t);
}
void go_Back(int t=0)  //Reverse
{
  digitalWrite(dir1PinL, LOW);
  digitalWrite(dir2PinL,HIGH);
  digitalWrite(dir1PinR,LOW);
  digitalWrite(dir2PinR,HIGH);
  analogWrite(speedPinL,100);
  analogWrite(speedPinR,100);
  delay(t);
}
void go_Back_Alert(int t=0)  //Reverse
{
  digitalWrite(dir1PinL, LOW);
  digitalWrite(dir2PinL,HIGH);
  digitalWrite(dir1PinR,LOW);
  digitalWrite(dir2PinR,HIGH);
  analogWrite(speedPinL,400);
  analogWrite(speedPinR,400);
  delay(t);
}



void stop_Stop()    //Stop
{
  digitalWrite(dir1PinL, LOW);
  digitalWrite(dir2PinL,LOW);
  digitalWrite(dir1PinR,LOW);
  digitalWrite(dir2PinR,LOW);
}

/**************detect IR code***************/
void do_IR_Tick()
{
  if(IR.decode(&IRresults))
  {
    if(IRresults.value==IR_ADVANCE)
    {
      Drive_Num=GO_ADVANCE;
    }
    else if(IRresults.value==IR_RIGHT)
    {
       Drive_Num=GO_RIGHT;
    }
    else if(IRresults.value==IR_LEFT)
    {
       Drive_Num=GO_LEFT;
    }
    else if(IRresults.value==IR_BACK)
    {
        Drive_Num=GO_BACK;
    }
    else if(IRresults.value==IR_STOP)
    {
        Drive_Num=STOP_STOP;
    }
    IRresults.value = 0;
    IR.resume();
  }
}

void x_bee()
{
  int alert = myNode.alertReceived();
  if (alert != AlertNode::NO_ALERT) {
    if (alert == AlertNode::ZOMBIE) {
      Serial.println("*** DANGER: Zombies !!! ");
      Drive_Num = GO_BACK;
      do_Drive_Tick();

    }
    if (alert == AlertNode::FIRE) {
      Serial.println("*** DANGER: FIRE !!! ");
      Drive_Num = GO_LEFT;
      do_Drive_Tick();

    }
    if (alert == AlertNode::FLOOD) {
      Serial.println("*** DANGER: FLOOD !!! ");
      Drive_Num = GO_RIGHT;
      do_Drive_Tick();

    }
}
}
/**************car control**************/
void do_Drive_Tick()
{
    switch (Drive_Num)
    {
      case GO_ADVANCE:go_Advance();JogFlag = true;JogTimeCnt = 1;JogTime=millis();break;//if GO_ADVANCE code is detected, then go advance
      case GO_LEFT: go_Left();JogFlag = true;JogTimeCnt = 1;JogTime=millis();break;//if GO_LEFT code is detected, then turn left
      case GO_RIGHT:  go_Right();JogFlag = true;JogTimeCnt = 1;JogTime=millis();break;//if GO_RIGHT code is detected, then turn right
      case GO_BACK: go_Back();JogFlag = true;JogTimeCnt = 1;JogTime=millis();break;//if GO_BACK code is detected, then backward
      case STOP_STOP: stop_Stop();JogTime = 0;break;//stop
      default:break;
    }
    Drive_Num=DEF;
   //keep current moving mode for  200 millis seconds
    if(millis()-JogTime>=200)
    {
      JogTime=millis();
      if(JogFlag == true)
      {
        stopFlag = false;
        if(JogTimeCnt <= 0)
        {
          JogFlag = false; stopFlag = true;
        }
        JogTimeCnt--;
      }
      if(stopFlag == true)
      {
        JogTimeCnt=0;
        stop_Stop();
      }
    }
}
int watch(){
  long howfar;
  digitalWrite(Trig_PIN,LOW);
  delayMicroseconds(5);
  digitalWrite(Trig_PIN,HIGH);
  delayMicroseconds(15);
  digitalWrite(Trig_PIN,LOW);
  howfar=pulseIn(Echo_PIN,HIGH);
  howfar=howfar*0.01657; //how far away is the object in cm
  //Serial.println((int)howfar);
  return round(howfar);
}

void auto_avoidance(){

  distance = watch(); // use the watch() function to see if anything is ahead (when the robot is just moving forward and not looking around it will test the distance in front)
  if (distance<distancelimit)
    { // The robot will just stop if it is completely sure there's an obstacle ahead (must test 25 times) (needed to ignore ultrasonic sensor's false signals)
      ++thereis;
    }
  if (distance>distancelimit){
      thereis=0;} //Count is restarted
  if (thereis > 10){
    thereis=0;
    stop_Stop();
    go_Back();
    go_Left();

  }
}

void setup()
{
  Serial.begin(9600);
  Serial.print("\n\n*** Starting HappyHome demo: ");
  Serial.println(myNodeName);
  Serial.print("*** system alarm = ");

  pinMode(dir1PinL, OUTPUT);
  pinMode(dir2PinL, OUTPUT);
  pinMode(speedPinL, OUTPUT);
  pinMode(dir1PinR, OUTPUT);
  pinMode(dir2PinR, OUTPUT);
  pinMode(speedPinR, OUTPUT);
  stop_Stop();

  pinMode(IR_PIN, INPUT);
  digitalWrite(IR_PIN, HIGH);
  IR.enableIRIn();
  /*init HC-SR04*/
  pinMode(Trig_PIN, OUTPUT);
  pinMode(Echo_PIN,INPUT);
  digitalWrite(Trig_PIN,LOW);

  myNode.setDebug(false);
  myNode.begin(myNodeName);

}

void loop()
{
  do_IR_Tick();
  do_Drive_Tick();
  x_bee();
  auto_avoidance();

}
