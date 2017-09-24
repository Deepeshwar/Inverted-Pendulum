#define encoder0PinA  18  //CLK Output A Do not use other pin for clock as we are using interrupt
#define encoder0PinB  19  //DT Output B

const int MotorPin = 10;
const int PwmPin   = 11;
volatile int count = 0;
int   IR_pin       = 3;
volatile boolean changeFlag = false;

#define midcount      60
#define Max           254

float kp=20 , kd = 0.1 , ki = 40 , E = 0 , e_old = 0;
volatile int encoder0Pos = 0;
float pid = 0;
float angle,rpm;
int cor;
float PID(float error)
{
  
  if((error>-0.5)&&(error<0.5))
   {
    E=0;
    rpm=0;
    cor=0;
    }
  E += error;
  if(E>300)
   {
    E=300;
    }
    else if(E<-300)
     {
      E =-300;
      }
  
  pid = (kp * error) + (ki * E) + (kd * (error - e_old));
   e_old = error;
  return pid;
}

void setup() { 

  pinMode(IR_pin , INPUT);
  pinMode(MotorPin,OUTPUT);
  pinMode(PwmPin, OUTPUT);
  pinMode(encoder0PinA, INPUT); 
  digitalWrite(encoder0PinA, HIGH);       // turn on pullup resistor
  pinMode(encoder0PinB, INPUT); 
  digitalWrite(encoder0PinB, HIGH);       // turn on pullup resistor
  attachInterrupt(digitalPinToInterrupt(18), doEncoderA, RISING); // encoder pin on interrupt 0 - pin2
  attachInterrupt(digitalPinToInterrupt(19), doEncoderB, FALLING); // encoder pin on interrupt 0 - pin2
  attachInterrupt(digitalPinToInterrupt(3), encoderINT5, RISING);
  Serial.begin (9600);
  Serial.println("start");                // a personal quirk
} 
 
void loop(){
// do some stuff here - the joy of interrupts is that they take care of themselves
  angle = encoder0Pos * 0.3;
 
  /*if(angle > 360 )
  {
    angle = angle - 360;
  }
  if(angle < -360 )
  {
    angle = angle + 360;
  }*/
  if(changeFlag)
   {
    changeFlag=false;
   }
   //if(count < 50 && count > -50)
   //{
    
    rpm = PID(angle);
    if(rpm > Max )
    {
      rpm=Max;
    }
    if(rpm < -Max)
    {
      rpm=-Max;
    }

  if(angle < 0)
  {
    digitalWrite(MotorPin,LOW);
    cor = -rpm;
  }
  else if(angle > 0)
  {
    digitalWrite(MotorPin,HIGH);
    cor = rpm;
  }
  analogWrite(PwmPin, cor);
    
  
   /*}
   else if(count > 50 || count < -50)
   {
    rpm =0;
    analogWrite(PwmPin, rpm);
   }*/
  
 /* Serial.print("Position:");
  Serial.print (angle);  //Angle = (360 / Encoder_Resolution) * encoder0Pos
  Serial.print("  ");
  Serial.print(rpm);
  Serial.print("  ");
  Serial.println(count);*/
  Serial.println(encoder0Pos);
  delay(10);
}
 
void doEncoderA() {
  if (digitalRead(encoder0PinB)==HIGH) {
    encoder0Pos++;
  } else {
    encoder0Pos--;
  }
}
void doEncoderB() {
  if (digitalRead(encoder0PinA)==HIGH) {
    encoder0Pos++;
  } else {
    encoder0Pos--;
  }
}

void encoderINT5()
{
  if(digitalRead(MotorPin) == HIGH)
  {
    count++;
  }
  else if(digitalRead(MotorPin) == LOW)
  {
    count--;
  } 
  changeFlag = true;
}
