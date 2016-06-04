//Radio Control PPM 
/* CH1 -- PIN2  (INT0)
 * CH2 -- PIN3  (INT1)
 * CH3 -- PIN21 (INT2)
 * CH4 -- PIN20 (INT3)
 */
#define __IO volatile

//Radio Controller Raw Value
__IO int rc1_val = 0;
__IO int rc2_val = 0;
__IO int rc3_val = 0;
__IO int rc4_val = 0;

//Pin number mapping
int ppm1 = 2;
int ppm2 = 3;
int ppm3 = 21;
int ppm4 = 20;

//Timer for ppm signal decoding
unsigned long rc1_pulseTick;
unsigned long rc2_pulseTick;
unsigned long rc3_pulseTick;
unsigned long rc4_pulseTick;

/*    EN IN+ IN-
 * LF 4  28 29
 * LR 5  30 31
 * RF 6  32 33
 * RR 7  34 35
 * 
 */
#define LF 0
#define LR 1
#define RF 2
#define RR 3
#define FORWARD 0
#define BACK    1

int MotorEN[4]={4,5,6,7};
int MotorINP[4]={29,30,33,34};
int MotorINM[4]={28,31,32,35};

void setup() {
  // put your setup code here, to run once:
  for(int i=0;i<4;i++)
  {
    motorCmd(i,0,FORWARD);
  }
  Serial.begin(9600);
  
  pinMode(ppm1,INPUT);
  pinMode(ppm2,INPUT);
  pinMode(ppm3,INPUT);
  pinMode(ppm4,INPUT);

  attachInterrupt(0,rc_isr1,CHANGE);
  attachInterrupt(1,rc_isr2,CHANGE);
  attachInterrupt(2,rc_isr3,CHANGE);
  attachInterrupt(3,rc_isr4,CHANGE);
  
  delay(500);

  while(isMidpos(rc1_val)!=true || isMidpos(rc2_val)!=true || isMidpos(rc4_val)!=true)
  {
    delay(20);
  }
}

boolean isMidpos(int rc_val)
{
  if( rc_val<1600 && rc_val>1400) return true;
  else return false;
}
//ISR 1~4
void rc_isr1(){
  if( digitalRead(ppm1) == HIGH )
  {
    rc1_pulseTick = micros();
  }
  else
  {
    rc1_val = micros() - rc1_pulseTick;
  }
}

void rc_isr2(){
  if( digitalRead(ppm2) == HIGH )
  {
    rc2_pulseTick = micros();
  }
  else
  {
    rc2_val = micros() - rc2_pulseTick;
  }
}

void rc_isr3(){
  if( digitalRead(ppm3) == HIGH )
  {
    rc3_pulseTick = micros();
  }
  else
  {
    rc3_val = micros() - rc3_pulseTick;
  }
}

void rc_isr4(){
  if( digitalRead(ppm4) == HIGH )
  {
    rc4_pulseTick = micros();
  }
  else
  {
    rc4_val = micros() - rc4_pulseTick;
  }
}

void motorCmd(int motorNum,int pwm,int dir)
{
 if(pwm < 50)
 {
   digitalWrite(MotorEN[motorNum],LOW);
   digitalWrite(MotorINP[motorNum],LOW);
   digitalWrite(MotorINM[motorNum],LOW);
 }
 else
 {
  analogWrite(MotorEN[motorNum],pwm);
  if(dir == FORWARD)
  {
   digitalWrite(MotorINP[motorNum],HIGH);
   digitalWrite(MotorINM[motorNum],LOW);
  }
  else
  {
   digitalWrite(MotorINP[motorNum],LOW);
   digitalWrite(MotorINM[motorNum],HIGH);    
  }
 }
}

//#define DEBUG_INFO
//#define MOTOR_TEST
void loop() {
  // put your main code here, to run repeatedly:
#ifdef DEBUG_INFO
  delay(200);
  Serial.print(rc1_val);Serial.print(", ");
  Serial.print(rc2_val);Serial.print(", ");
  Serial.print(rc3_val);Serial.print(", ");
  Serial.print(rc4_val);Serial.println(".");
#endif
#ifdef MOTOR_TEST
  int i;
  for(i=0;i<4;i++)
  {
    motorCmd(i,200,FORWARD);
    delay(1000);
    motorCmd(i,0,FORWARD);
    delay(1000);
    motorCmd(i,200,BACK);
    delay(1000);
    motorCmd(i,0,BACK);
    delay(1000);    

  }
#endif
  int speed = 100;
  if( rc2_val < 1200)
  {
    motorCmd(LF,speed,FORWARD);
    motorCmd(LR,speed,FORWARD);
    motorCmd(RF,speed,FORWARD);
    motorCmd(RR,speed,FORWARD);
  }
  else if(rc2_val > 1800)
  {
    motorCmd(LF,speed,BACK);
    motorCmd(LR,speed,BACK);
    motorCmd(RF,speed,BACK);
    motorCmd(RR,speed,BACK);
  }
  else if( rc4_val < 1200)
  {
    motorCmd(LF,speed,BACK);
    motorCmd(LR,speed,FORWARD);
    motorCmd(RF,speed,FORWARD);
    motorCmd(RR,speed,BACK);
  }
  else if(rc4_val > 1800)
  {
    motorCmd(LF,speed,FORWARD);
    motorCmd(LR,speed,BACK);
    motorCmd(RF,speed,BACK);
    motorCmd(RR,speed,FORWARD);
  }
  else if( rc1_val < 1200)
  {
    motorCmd(LF,speed,BACK);
    motorCmd(LR,speed,BACK);
    motorCmd(RF,speed,FORWARD);
    motorCmd(RR,speed,FORWARD);
  }
  else if(rc1_val > 1800)
  {
    motorCmd(LF,speed,FORWARD);
    motorCmd(LR,speed,FORWARD);
    motorCmd(RF,speed,BACK);
    motorCmd(RR,speed,BACK);
  }
  else
  {
    motorCmd(LF,0,FORWARD);
    motorCmd(LR,0,FORWARD);
    motorCmd(RF,0,FORWARD);
    motorCmd(RR,0,FORWARD);
  }
  delay(200);
}
