//Radio Control PPM 
/* CH1 -- PIN2  (INT0)
 * CH2 -- PIN3  (INT1)
 * CH3 -- PIN21 (INT2)
 * CH4 -- PIN20 (INT3)
 */
#define __IO volatile

//Radio Controller Raw Value
__IO int rc1_val;
__IO int rc2_val;
__IO int rc3_val;
__IO int rc4_val;

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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  pinMode(ppm1,INPUT);
  pinMode(ppm2,INPUT);
  pinMode(ppm3,INPUT);
  pinMode(ppm4,INPUT);

  attachInterrupt(0,rc_isr1,CHANGE);
  attachInterrupt(1,rc_isr2,CHANGE);
  attachInterrupt(2,rc_isr3,CHANGE);
  attachInterrupt(3,rc_isr4,CHANGE);
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

void loop() {
  // put your main code here, to run repeatedly:
  delay(200);
  Serial.print(rc1_val);Serial.print(", ");
  Serial.print(rc2_val);Serial.print(", ");
  Serial.print(rc3_val);Serial.print(", ");
  Serial.print(rc4_val);Serial.println(".");
}
