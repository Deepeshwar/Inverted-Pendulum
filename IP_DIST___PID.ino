
#define   F_CPU     16000000UL
#define   T_count   65286

#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

#define encoder0PinA   18  //CLK Output A Do not use other pin for clock as we are using interrupt
#define encoder0PinB   19  //DT Output B
#define position_encA  20
#define position_encB  21
#define lin_setpoint   0
 
const int MotorPin =  10;
const int PwmPin   =  11;
const int ledPin   =  13;

#define vel           OCR1A
#define Max           254
#define Max_ang       3
#define max_size      5

volatile int  TICK=0;
volatile long TICK1=0;
volatile double angle=0.0;

void pwm_init(void)
{
  TCCR1A |= (1 << COM1A1) | (1 << WGM10);
  TCCR1B |= (1 << WGM12) | (1 << CS10);
}

//float kp=80 , kd = 0.05 , ki = 0.5 , E_rot = 0 , e_old_rot = 0;
float kp=20 , kd = 0 , ki = 0.1 , E_rot = 0 , e_old_rot = 0;
float kp1=0.1 , kd1 = 0 , ki1 = 0.5 , E_lin = 0 , e_old_lin = 0;

float pid_rot = 0, pid_lin = 0, Gain[6]={kp , kd , ki , kp1 , kd1 , ki1};
float rpm , rot_setpoint=0 , Err_angle = 0 , Temp1=0 , dist = 0.0, Err_dist = 0;
int cor , i=0 ,flag=0,j=0,index=0,m=0;
float Avg_angle=0.0,fin_ang=0.0,movArray[10]={0.0};
float Avg_dis=0.0,fin_dis=0.0,disArray[10]={0.0};

float PID_rot(float error_rot)
{
   if(error_rot == 0)
   {
    E_rot=0;
    rpm=0;
    cor=0;
    }
  E_rot += error_rot;
  if(E_rot>300)
   {
    E_rot=300;
    }
    else if(E_rot<-300)
     {
      E_rot =-300;
      }
  
  pid_rot = (kp * error_rot) + (ki * E_rot) + (kd * (error_rot - e_old_rot));
   e_old_rot = error_rot;
  return pid_rot;
}

float PID_lin(float error_lin)
{
   if(error_lin == 0)
   {
    E_lin=0;
    pid_lin = 0;
    }
    
  E_lin += error_lin;
  
  if(E_lin > 10)
   {
    E_lin=10;
    }
    else if(E_lin<-10)
     {
      E_lin =-10;
      }
  
  pid_lin = (kp1 * error_lin) + (ki1 * E_lin) + (kd1 * (error_lin - e_old_lin));
   e_old_lin = error_lin;

   if(pid_lin > Max_ang)
   {
    pid_lin = Max_ang;
   }
   else if(pid_lin < -Max_ang)
   {
    pid_lin = -Max_ang;
   }
  return pid_lin;
}

char Value;

void setup() {
  
      
        pinMode(MotorPin,OUTPUT);
        pinMode(PwmPin, OUTPUT);
        pinMode(ledPin, OUTPUT);
        
        pinMode(position_encA , INPUT);
        digitalWrite(position_encA, HIGH);
        pinMode(position_encB , INPUT);
        digitalWrite(position_encB, HIGH);
        pinMode(encoder0PinA, INPUT); 
        digitalWrite(encoder0PinA, HIGH);       // turn on pullup resistor
        pinMode(encoder0PinB, INPUT); 
        //digitalWrite(encoder0PinB, HIGH);       // turn on pullup resistor
        attachInterrupt(digitalPinToInterrupt(18), doEncoderA, RISING); // encoder pin on interrupt 0 - pin2
        attachInterrupt(digitalPinToInterrupt(19), doEncoderB, FALLING); // encoder pin on interrupt 0 - pin2
        attachInterrupt(digitalPinToInterrupt(20), encoderINTA, RISING);
        //attachInterrupt(digitalPinToInterrupt(21), encoderINTB, FALLING);

        pwm_init(); 

        sei();
        TCCR3A = 0X00;
        TCCR3B |= (1<<CS31) | (1<<CS30);
        TIMSK3 |= (1<<TOIE3);
        TCNT3 = T_count;
        //interrupts();
        
        Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps
}

void loop() {

  if(Serial.available() > 0)
  {
    while(Serial.available() > 0)
    {
      Gain[i] = Serial.read();
      i++;
        
      if(i>5)
      {
        i=0;
      }
    }
  }

  kp=Gain[0];
  kd=Gain[1] / 100;
  ki=Gain[2] / 100;
  
  kp1=Gain[3];
  kd1=Gain[4] / 100;
  ki1=Gain[5] / 100;

  
  if(dist > -45 && dist < 45)
  {
   // if(angle < Max_ang && angle > (-Max_ang))
    //{
      Err_dist = lin_setpoint - dist;
      
      Temp1 = PID_lin(Err_dist);

      rot_setpoint =Temp1;
      
      Err_angle = rot_setpoint - fin_ang;
      
      rpm = PID_rot(-Err_angle);
    //}
  }
  
    if(rpm > Max )
    {
      rpm=Max;
    }
    if(rpm < -Max)
    {
      rpm=-Max;
    }

  if(fin_ang < 0)
  {
    digitalWrite(MotorPin,LOW);
    cor = -rpm;
  }
  else if(fin_ang > 0)
  {
    digitalWrite(MotorPin,HIGH);
    cor = rpm;
  }

  if(dist > 45 || dist < -45)
  {
    cor=0;
  }
  //analogWrite(PwmPin, cor);
  vel = cor;

if(flag == 1)
{
  /*Serial.print(rot_setpoint);
  Serial.print(" ");
  Serial.print(fin_ang);
  Serial.print(" ");
  Serial.print(dist);
  Serial.print(" ");
  Serial.print(lin_setpoint);
  Serial.print(" ");*/
  
  flag=0;
}
  //Serial.println(fin_ang);
  //Serial.print(angle);
  //Serial.print(" ");
  Serial.print("Dist ");
  Serial.println(fin_dis);
  /*Serial.print(" ");
  Serial.print("rpm ");
  Serial.print(rpm);
  Serial.print(" ");
  Serial.print("Rot_setpoint ");
  Serial.println(Temp1);*/
  
  
  delay(5);

}

void doEncoderA() {
  if (digitalRead(encoder0PinB)==HIGH) {
    TICK++;
  } else {
    TICK--;
  }
}
void doEncoderB() {
  if (digitalRead(encoder0PinA)==HIGH) {
    TICK++;
  } else {
    TICK--;
  }
}

void encoderINTA()
{
 /* if(bit_is_set(PIND,2))
  {
    TICK1++;
  }
  else if(bit_is_clear(PIND,2))
  {
    TICK1--;
  } */
  if(digitalRead(position_encB) == HIGH)
  {
    TICK1++;
  }
  else if(digitalRead(position_encB) == LOW)
  {
    TICK1--;
  } 
}
/*void encoderINTB()
{
  if(digitalRead(position_encA) == HIGH)
  {
    TICK1++;
  }
  else if(digitalRead(position_encA) == LOW)
  {
    TICK1--;
  } 
}
*/
void addElement(float element,float element1)
{
  /*total -= movArray[index];
  movArray[index]=element;
  total+=element;
  index++;
  if(index == max_size)
  {
    index=0;
  }*/
  movArray[0] = element;
  disArray[0] = element1;
  
  for(index=max_size; index>1; index--)
   {
    movArray[index-1] = movArray[index-2];
    disArray[index-1] = disArray[index-2];
    }
   for(index=0; index<max_size; index++)
     {
        Avg_angle += movArray[index];
        Avg_dis   += disArray[index];
      }
     Avg_angle =  Avg_angle/max_size; 
     Avg_dis = Avg_dis/max_size;
     fin_dis  = Avg_dis;
     Avg_dis =0;
      fin_ang = Avg_angle;
       Avg_angle= 0;
}

ISR(TIMER3_OVF_vect)
{
  dist = TICK1 * 0.0084;
  angle = TICK * 0.3;
  addElement(angle,dist);
  
  
  m++;
  if(m==100)
   {
     flag=1;
     m=0; 
   }
  TCNT3 = T_count;
}

