#include <Arduino.h>
#include <Encoder.h>
#include <QTRSensors.h>
#include <LiquidCrystal.h>

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

int leftIR = 0;
int rightIR = 0;
int junc = 0;

int turnDis = 600;
int ktd = 520;

// const int baseSpeed = 80;
// const int maxSpeed = 160;



const int leftPWM = 8;
const int rightPWM = 9;
const int leftF  = 52;
const int leftB = 50;
const int rightF = 48;
const int rightB = 46;


const int echo =  6;
const int trig =  7;

const int echoR =  42;
const int trigR =  44;

const int echoL =  15;
const int trigL =  14;


const int setPoint = 7500;


const int turnSpeed = 100;

//const double Kp =0.016333;
//const double Kd =0.1;
//const double Ki =0;//.001;

int totalErr = 0;
int prevError = 0;


Encoder right(18, 19);
Encoder left(20,21);

QTRSensors qtr;

const uint8_t SensorCount = 16;
uint16_t sensorValues[SensorCount];

const int btnPin = 38;

int c = 0;

void setup()
{

  pinMode(leftPWM,OUTPUT);
  pinMode(rightPWM,OUTPUT);
  pinMode(leftF,OUTPUT);
  pinMode(leftB,OUTPUT);
  pinMode(rightF,OUTPUT);
  pinMode(rightB,OUTPUT);

  pinMode(btnPin,INPUT_PULLUP);
  pinMode(36,INPUT_PULLUP);
  pinMode(40,INPUT_PULLUP);
   pinMode(42,INPUT_PULLUP);


  digitalWrite(leftF,HIGH);
  digitalWrite(leftB,LOW);
  digitalWrite(rightF,HIGH);
  digitalWrite(rightB,LOW);
  
  qtr.setTypeRC();                   
  qtr.setSensorPins((const uint8_t[]){37,39,41,43,45,47,49,51,35,33,31,29,27,25,23,22}, SensorCount);
  lcd.begin(16,2);
  lcd.setCursor(0, 0);
  lcd.print("calibrating      ");
  delay(500);
  //calibrate();
  stop();
  lcd.setCursor(0, 0);
  lcd.print("done            ");
  stop();
  delay(2000);
  lcd.setCursor(0, 0);
  lcd.print("line following  ");
  lcd.setCursor(0,1);

  pinMode(trigL,OUTPUT);
  pinMode(echoL,INPUT);

  Serial.begin(9600);
  Serial.println("test");
}

 int baseSpeed = 70;
 int maxSpeed = 110;

 const int Kbs = 80;
 const int Kms = 160;

const double Kp =0.02033;
const double Kd =0.4;
const double Ki =0;//.001;

void loop()
{   
    turnBack();
    stop();
    while(true)
    {}
}

int backIR(){
  int readings[4];
  readings[0] = analogRead(A0);
  readings[1] = analogRead(A1);
  readings[2] = analogRead(A2);
  readings[3] = analogRead(A3);
  
  for(int i=0;i<4;i++){
    if(readings[i]<= 500)
      return 1;
  }
  
  return 0;
}

void wallFollow(){
  int dis = leftDistance();
  lcd.setCursor(0,0);
  lcd.print("wall following  ");
  lcd.setCursor(0,1);
  lcd.print("       ");
  lcd.setCursor(0,1);
  lcd.print(dis);
  int error = 3 - dis;
  int correction = error *20;
  int bSpeed = 170;
  leftForward(bSpeed+correction);
  rightForward(bSpeed-correction);
  delay(20);
  stop();
  delay(2);
}

void partI(){
  while(true){
      int error = getErr();
      if(leftIR==1 || rightIR==1){
      stop();
      delay(200);
      leftForward(100);
      rightForward(100);
      delay(20);
      stop();
      delay(100);
      setupEncoders();
      if(leftIR==0 && rightIR==1)
        turnRight();
      else if(leftIR==1 && rightIR==0)
        turnLeft();
      else if(leftIR==1 && rightIR==0)
        turnBack();
      continue;
    }else if((16-c)>=9 ){
      stop();
      delay(200);
      turnRight60();

      return;
    }
    pid(error);
  }
}

void partII(){
    int error = getErr();

    baseSpeed = 70;
    maxSpeed = 110;

    while (true)
    {
      if(((16-c)>=9)){
        // setupEncoders();
        // stop();
        // delay(1000);
        // for(int i=0;i<100;i++){
        //   forwardP();
        //   delay(2);
        // }
        // stop();
        // delay(1000);
        // getErr();
        // if(leftIR==1 || rightIR==1)
          break;
        
        // int ll = 0;
        // int rr = 0;
        // for(int i=0;i<200;i++){
        //   leftForward(100);
        //   rightForward(100);
        //   getErr;
        //   if(ll==0)ll=leftIR;
        //   if(rr==0)rr=rightIR;

        //   if(ll==1 && rr==1)
        //     break;
        // }
      }
      followDash(error);
      error = getErr();
    }

    baseSpeed = Kbs;
    maxSpeed = Kms;
    stop();
    delay(100);
    setupEncoders();
    for(int i=0;i<100;i++){
        forwardP();
        delay(1);
    }
    stop();
    delay(100);
    turnRight60();
    return;
}



void partIII(){
  while(true){
    if(getDistance()<=7){
      lcd.setCursor(0,1);
      lcd.print(String(getDistance())+"     ");
      stop();
      delay(800);
      //check color/////////////////////
      
      stop();
      delay(400);
      // maxSpeed = 100;
      // for(int i=0;i<50;i++){
      //   int error = getErr();
      //   setupPosition(error);
      // }
      // maxSpeed = Kms;
      stop();
      delay(200); 
      turnDis = 600;
      turnLeft();
      stop();
      delay(500);
      cylinderLeft();
      stop();
      delay(500);
      turnLeft();
      turnDis = ktd;
    }
    maxSpeed = 120;
    baseSpeed = 80;
    int error = getErr();

    if(leftIR==1 || rightIR==1){
      lcd.setCursor(0,0);
      lcd.print("              ");
      lcd.setCursor(0,0);
      lcd.print(String(leftIR)+" "+String(rightIR));
      stop();
      delay(200);
      setupEncoders();
      for(int i=0;i<100;i++){
        forwardP();
        delay(1);
      }
      delay(20);
      stop();
      stop();
      delay(200);
      setupEncoders();
      if(leftIR==0 && rightIR==1){
        getErr();
        if(c!=16)
          pickCoin(0);
        else
          turnRight();
      }
      else if(leftIR==1 && rightIR==0){
        getErr();
        if(c!=16)
          pickCoin(1);
        else
          turnLeft();
      }
    }else
    {
      pid(error);
    }
    
  }
}


void pickCoin(int i){
    if(i==0)
      turnLeft();
    else 
      turnRight();

    int error = getErr();
    while(c==0){
      pid(error);
      error = getErr();
    }
    ///////////////////check coin
    stop();
    delay(100);

    turnBack();

    stop();
    delay(100);

    error = getErr();
    while(!(16-c >= 10)){
      pid(error);
      error = getErr();
    }
    //////////////////////////////////////////////////////////////forward
    if(i==0)
      lineFollow('r');
    else
      lineFollow('l');  
}

void lineFollow(char choice){
  int error = getErr();
  if(leftIR==1 || rightIR==1){
    if(leftIR==1 && rightIR==0){
      setupEncoders();
      for(int i=0;i<60;i++){
        forwardP();
        delay(1);
      }
      turnLeft();
    }else if(leftIR==0 && rightIR==1){
      setupEncoders();
      for(int i=0;i<60;i++){
        forwardP();
        delay(1);
      }
      turnRight();
    }else
    {
      setupEncoders();
      for(int i=0;i<60;i++){
        forwardP();
        delay(1);
      }
      if(choice=='l')
        turnLeft();
      else
        turnRight();
    }
    
  }else
  pid(error);
  
}


void followDash(int err){
    if(c!=0){
        pid(err);
    }else{
        leftForward(baseSpeed);
        rightForward(baseSpeed);
    }
}

int getSideDisL(){
      digitalWrite(trig, LOW);
      delayMicroseconds(2);

      digitalWrite(trig, HIGH);
      delayMicroseconds(10);
      digitalWrite(trig, LOW);

      double duration;
      int distance;

      duration =  pulseIn(echo, HIGH);
      
      distance = (double)duration * 0.034 / 2.0;
    
      return distance;
}

const double x = 22.0/7.0/2.0*25.0*(938.0/23.0);
const double y = 22.0/7.0/2.0*45.0*(938.0/23.0);
const double Kf = 200;
void cylinderRight(){
  int speed = 200;
  double xx = x/Kf;
  double yy = y/Kf;
  lcd.setCursor(0,1);
  lcd.print(String(xx));
  lcd.print(" ");
  lcd.print(String(yy));

  setupEncoders();
  int leftRead = left.read();
  int rightRead = right.read();
  for(int i=0;i<Kf;i++){
    lcd.print("               ");
    while(rightRead<=xx){
      leftForward(speed);

      rightRead = right.read();
      rightRead = rightRead*-1;
      lcd.setCursor(0,1);
      lcd.print(String(rightRead));
    }
    stop();
    while(leftRead<=yy){
      rightForward(speed);

      leftRead = left.read();
      leftRead = leftRead*-1;
      lcd.setCursor(0,1);
      lcd.print(String(leftRead));
    }
    stop();
    xx+=x/Kf;
    yy+=y/Kf;
    
  }
  stop();
  delay(100);
  setupEncoders();
  // for(int i=0;i<35;i++){
  //   getErr();
  //   if((16-c)>=4)
  //     break;
  //   backwardP();
  //   delay(1);
  // }
  // stop();
  delay(400);
}

void cylinderLeft(){
  int speed = 200;
  double xx = x/Kf;
  double yy = y/Kf;
  lcd.setCursor(0,1);
  lcd.print(String(xx));
  lcd.print(" ");
  lcd.print(String(yy));
  setupEncoders();
  int leftRead = left.read();
  int rightRead = right.read();
  for(int i=0;i<2.18*Kf;i++){
    lcd.print("               ");
    while(rightRead<=yy){
      leftForward(speed);

      rightRead = right.read();
      rightRead = rightRead*-1;
    }
    stop();
    while(leftRead<=xx){
      rightForward(speed);

      leftRead = left.read();
      leftRead = leftRead*-1;
    }
    stop();
    xx+=x/Kf;
    yy+=y/Kf;
    
  }
  stop();
  delay(400);

  // for(int i=0;i<1500;i++){
  //   rightForward(80);
  //   leftForward(130);
  //   delay(1);
  // }
  
}

void pid(int err){
    int p = err * Kp;
    totalErr += (err/100);
    int i = totalErr*Ki;
    int d = (err - prevError) * Kd;
    prevError = err;
    int correction = p+i+d;
    if(correction>0){
      //printTuple(baseSpeed-correction, baseSpeed+correction);
      leftForward(baseSpeed-correction);
      rightForward(baseSpeed+correction);
    }else{
      correction = correction * -1;
      //printTuple(baseSpeed+correction,baseSpeed-correction);
      rightForward(baseSpeed-correction);
      leftForward(baseSpeed+correction);
    }
   
}

int getErr(){
    int position = qtr.readLineBlack(sensorValues);
    int p = position;
    int vals[16];
    int sum;
    int count = 0;
    for(int i=0;i<16;i++){
        if(sensorValues[i]>500)
        {
            vals[i] = 0; 
        }
        else
        {
            vals[i] = 1;
            count++;
        }
        sum += i+1;

    }
    String reading = "";
    for(int i=15;i>=0;i--){
      if(vals[i]==0)
        reading+="0";
      else
        reading+="1";
    }
    boolean b = vals[6]==0 || vals[7]==0 || vals[8]==0;
    if(b){
      leftIR = vals[0]==1?0:1;
      rightIR = vals[15]==1?0:1;
    }else{
      leftIR = 0;
      rightIR = 0;
    }
 
    junc = leftIR + rightIR;
    c = count;
    position = (sum*1000)/count;
    lcd.setCursor(0,1);
    lcd.print(reading);
    return setPoint - p;
}


void calibrate(){
    
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  leftForward(1);
  rightBackward(1);
  for (uint16_t i = 0; i < 50; i++)
  {
    analogWrite(leftPWM,100);
    analogWrite(rightPWM,100);
    qtr.calibrate();
  }

  leftBackward(1);
  rightForward(1);

  for (uint16_t i = 0; i < 55; i++)
  {
    analogWrite(leftPWM,100);
    analogWrite(rightPWM,100);
    qtr.calibrate();
  }

  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

int leftDistance()
{
  digitalWrite(trigL, LOW);
  delayMicroseconds(2);

  digitalWrite(trigL, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigL, LOW);

  double duration;
  int distance;

  duration =  pulseIn(echoL, HIGH);
  
  distance = (double)duration * 0.034 / 2.0;
 
  return distance;
}

int rightDistance()
{
  digitalWrite(trigR, LOW);
  delayMicroseconds(2);

  digitalWrite(trigR, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigR, LOW);

  double duration;
  int distance;

  duration =  pulseIn(echoR, HIGH);
  
  distance = (double)duration * 0.034 / 2.0;
 
  return distance;
}

int getDistance()
{
  digitalWrite(trig, LOW);
  delayMicroseconds(2);

  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  double duration;
  int distance;

  duration =  pulseIn(echo, HIGH);
  
  distance = (double)duration * 0.034 / 2.0;
 
  return distance;
}


void forwardP(){
  int leftR = left.read();
  int rightR = right.read();
  leftR = leftR;
  rightR = rightR;
  int error = leftR-rightR;
  int correction = error * 20;
  int speed = 100;
  leftForward(speed-correction);
  rightForward(speed+correction);
}


void backwardP(){
  int leftR = left.read();
  int rightR = right.read();
  leftR = leftR*-1;
  rightR = rightR*-1;
  int error = leftR-rightR;
  int correction = error * 20;
  int speed = 100;
  leftBackward(speed-correction);
  rightBackward(speed+correction);
}

void turnLeft60(){
  setupEncoders();
  int reading = right.read();
  while((reading)<200){
    rightForward(turnSpeed);
    leftBackward(turnSpeed);
    reading = right.read();
  }
}

void turnRight60(){
  setupEncoders();
  int reading = left.read();
  while((reading)<200){
    leftForward(turnSpeed);
    rightBackward(turnSpeed);
    reading = left.read();
  }
}

void turnLeft(){
  setupEncoders();
    int reading = right.read();
  int initial = reading;
  while((reading)<turnDis){
    rightForward(turnSpeed);
    leftBackward(turnSpeed);
    reading = right.read();
  }
}


void turnRight(){
  setupEncoders();
  int reading = left.read();
  while((reading)<turnDis){
    leftForward(turnSpeed);
    rightBackward(turnSpeed);
    reading = left.read();
  }
 
}

void turnBack(){
  setupEncoders();
  int reading = left.read();
  while((reading)<1300){
    leftForward(turnSpeed);
    rightBackward(turnSpeed);
    reading = left.read();
    lcd.setCursor(0,0);
    lcd.print("             ");
    lcd.print(right.read());
  }
  stop();
}

void leftForward(int speed){
  if(speed<0)
    speed = 0;
  else if(speed>maxSpeed)
    speed = maxSpeed;

  digitalWrite(leftF,HIGH);
  digitalWrite(leftB,LOW);
  analogWrite(leftPWM,speed);
}

void rightForward(int speed){
  if(speed<0)
    speed = 0;
  else if(speed>maxSpeed)
    speed = maxSpeed;

  digitalWrite(rightF,HIGH);
  digitalWrite(rightB,LOW);
  analogWrite(rightPWM,speed);
}

void leftBackward(int speed){
  if(speed<0)
    speed = 0;
  else if(speed>maxSpeed)
    speed = maxSpeed;

  digitalWrite(leftF,LOW);
  digitalWrite(leftB,HIGH);
  analogWrite(leftPWM,speed);
}

void rightBackward(int speed){
  if(speed<0)
    speed = 0;
  else if(speed>maxSpeed)
    speed = maxSpeed;

  digitalWrite(rightF,LOW);
  digitalWrite(rightB,HIGH);
  analogWrite(rightPWM,speed);
}


void setupEncoders(){
  left.write(0);
  right.write(0);
}

void printTuple(int a , int b){
  Serial.print(a);
  Serial.print("\t");
  Serial.println(b);
}

void stop(){
  digitalWrite(leftF,HIGH);
  digitalWrite(leftB,HIGH);
  digitalWrite(rightF,HIGH);
  digitalWrite(rightB,HIGH);
  analogWrite(leftPWM,255);
  analogWrite(rightPWM,255);
}

void setupPosition(int err){
    if(err<0)
      err *= (16-c)/4;
    else
      err *= (16-c)/4;
    
    int p = err * 0.1;
    int correction = p;
    if(correction>0){
      //printTuple(baseSpeed-correction, baseSpeed+correction);
      leftBackward(correction);
      rightForward(correction);
    }else{
      correction = correction * -1;
      //printTuple(baseSpeed+correction,baseSpeed-correction);
      rightBackward(correction);
      leftForward(correction);
    }
   
}


