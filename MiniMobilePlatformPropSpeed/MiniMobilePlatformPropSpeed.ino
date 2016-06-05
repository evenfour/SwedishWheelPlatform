#define __IO volatile

//Radio Control PPM 
/* CH1 -- PIN2  (INT0)
 * CH2 -- PIN3  (INT1)
 * CH3 -- PIN21 (INT2)
 * CH4 -- PIN20 (INT3)
 */
//Radio Controller Raw Value
__IO int rc1_val = 0;
__IO int rc2_val = 0;
__IO int rc3_val = 0;
__IO int rc4_val = 0;
//Pin number mapping
int ppm1 = 2;  // Interrupt 0
int ppm2 = 3;  // Interrupt 1
int ppm3 = 21; // Interrupt 2
int ppm4 = 20; // Interrupt 3
//Timer for ppm signal decoding
unsigned long rc1_pulseTick;
unsigned long rc2_pulseTick;
unsigned long rc3_pulseTick;
unsigned long rc4_pulseTick;
//Locomotion
double tx = 0;
double ty = 0;
double rz = 0;
double motorVector[4] = {0,0,0,0};
int motorPWM[4] = {0,0,0,0};
#define ServoMax 500
#define PWMMax   255
#define PWM_DEADZONE 20
double ServoMid[4]={1510,1510,1510,1510};

/* Power Input: Li-Po 2S 7.4V  ------->  Motor Driver
 *                             --LDO-->  5V Arduino ----> RC Receiver
 * Each motor has three control pins
 * On L298N Driver side:
 * White -- EN   Black and Read for IN+/-
 * On Arduino Mega 2560 side:   
 *    EN IN+ IN-
 * LF 4  29  28   Brown   Orange   Red
 * LR 5  30  31   Yellow  Green    Blue
 * RF 6  33  32   Purple  White    Gray
 * RR 7  34  35   Black   Brown    Red
 * 
 */
#define LF 0 //left front
#define LR 1 //left rear
#define RF 2 //right front
#define RR 3 //right rear
#define FORWARD 0 //towards platform head
#define BACK    1 //towards platform tail
// Motor control pins' list
int MotorEN[4]={4,5,6,7};
int MotorINP[4]={29,30,33,34};
int MotorINM[4]={28,31,32,35};

////////////////////////////////////////////////////////
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

  //check stick middle position for safty
  while(isMidpos(rc1_val)!=true || isMidpos(rc2_val)!=true || isMidpos(rc4_val)!=true)
  {
    delay(50);
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
 if(pwm < PWM_DEADZONE)
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
#define FACTOR PWMMax/ServoMax
void loop() {
  // put your main code here, to run repeatedly:
#ifdef DEBUG_INFO
  delay(200);
  Serial.print(rc1_val);Serial.print(", ");
  Serial.print(rc2_val);Serial.print(", ");
  Serial.print(rc3_val);Serial.print(", ");
  Serial.print(rc4_val);Serial.println(".");
#endif
  tx =  ServoMid[1] - rc2_val;
  ty =  ServoMid[3] - rc4_val;
  rz =  ServoMid[0] - rc1_val;

  if( abs(tx) < 10 ) tx=0;
  if( abs(ty) < 10 ) ty=0;
  if( abs(rz) < 10 ) rz=0;
  
  /////TX//////////
  for(int i=0;i<4;i++)
  {
    motorVector[i] = tx*FACTOR;
  }
  
  /////TY//////////
  {
    motorVector[LF] = motorVector[LF] - ty*FACTOR;
    motorVector[LR] = motorVector[LR] + ty*FACTOR;
    motorVector[RF] = motorVector[RF] + ty*FACTOR;
    motorVector[RR] = motorVector[RR] - ty*FACTOR;
  }
  
  /////RZ//////////
  {
    if( tx > -10 )
    {
      motorVector[LF] = motorVector[LF] - rz*FACTOR;
      motorVector[LR] = motorVector[LR] - rz*FACTOR;
      motorVector[RF] = motorVector[RF] + rz*FACTOR;
      motorVector[RR] = motorVector[RR] + rz*FACTOR;
    }
    else
    {
      motorVector[LF] = motorVector[LF] + rz*FACTOR;
      motorVector[LR] = motorVector[LR] + rz*FACTOR;
      motorVector[RF] = motorVector[RF] - rz*FACTOR;
      motorVector[RR] = motorVector[RR] - rz*FACTOR;      
    }
  }
  
  for(int i=0;i<4;i++)
  {
    int pwm = abs((int)motorVector[i]);
    int dir;
    if(motorVector[i] > 0)
    {
      dir = FORWARD;
    }
    else
    {  
      dir = BACK;
    }
    if( pwm > 255 ) 
    {
      pwm = 255;
    }
    else if( pwm < PWM_DEADZONE )  
    {
      pwm = 0;
    }
    motorCmd(i,pwm,dir);
  }
  delay(100);
}
