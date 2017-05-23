//pid
//1.Start with Kp, Ki and Kd equalling 0 and work with Kp first. Try setting Kp to a value of 1 and observe the robot. 
//The goal is to get the robot to follow the line even if it is very wobbly. If the robot overshoots and loses the line, reduce the Kp value.
//If the robot cannot navigate a turn or seems sluggish, increase the Kp value.
//2.Once the robot is able to somewhat follow the line, assign a value of 1 to Kd (skip Ki for the moment).
//Try increasing this value until you see less wobble.
//3.Once the robot is fairly stable at following the line, assign a value of .5 to 1.0 to Ki. If the Ki value is too high, the robot will jerk left and right quickly.
//If it is too low, you won't see any perceivable difference.  Since Integral is cumulative, the Ki value has a significant impact. You may end up adjusting it by .01 increments.
//4.Once the robot is following the line with good accuracy, you can increase the speed and see if it still is able to follow the line.
//Speed affects the PID controller and will require retuning as the speed changes.
//float Kp=15,Ki=0.6,Kd=2;
//float Kp=12,Ki=0,Kd=5;  
//float Kp=30,Ki=0.6,Kd=8 for 5 v;

  
int led=13,value=1;
float  Kp=30,Ki=0.5,Kd=15;
float error=0, P=0, I=0, D=0, PID_value=0;
float previous_error=0, previous_I=0;
int sensor[8]={0, 0, 0, 0, 0,0,0,0};
int sensor1[8]={0, 0, 0, 0, 0,0,0,0};
int sensor2[8]={0, 0, 0, 0, 0,0,0,0};
int sensor3[8]={0, 0, 0, 0, 0,0,0,0};
//int initial_motor_speed=125;

int initial_motor_speed=255;
int near=0;

void serialReading(void);
void read_sensor_values(void);
void calculate_pid(void);
void motor_control(void);

const byte interruptPin = 2;
volatile byte state = LOW;

void setup()
 {
  pinMode(A0,INPUT);
 pinMode(A1,INPUT);
 pinMode(A2,INPUT);//center 
 pinMode(A3,INPUT);
 pinMode(A4,INPUT);
 pinMode(A5,INPUT);//clip
 pinMode(12,INPUT);//near
 
pinMode(13,OUTPUT);

 pinMode(9,OUTPUT); //Left Motor Pin 1
 pinMode(10,OUTPUT); //Left Motor Pin 2 on forwar
 pinMode(5,OUTPUT); //Right Motor Pin 1
pinMode(6,OUTPUT);  //Right Motor Pin 2 on forward


 //pinMode(interruptPin, INPUT_PULLUP);
//attachInterrupt(digitalPinToInterrupt(interruptPin), blink, CHANGE);

 Serial.begin(9600); //Enable Serial Communications
 // digitalWrite(11,HIGH);
 //  digitalWrite(12,LOW);
 //  digitalWrite(9,HIGH);
 //  digitalWrite(10,LOW);
}
/*
void left_forward()
{
  digitalWrite(11,HIGH);
   digitalWrite(12,LOW);
}
void right_forward()
{
  digitalWrite(9,HIGH);
   digitalWrite(10,LOW);
}
void stop()
{digitalWrite(9,LOW);
   digitalWrite(10,LOW);
   digitalWrite(11,LOW);
   digitalWrite(12,LOW);
}
void forward_motor()
{ left_forward();
right_forward();

}
void right_f()
{
  digitalWrite(9,HIGH);
   digitalWrite(10,LOW);
   digitalWrite(11,0);
   digitalWrite(12,0);
}
void left_f()
{
  digitalWrite(9,0);
   digitalWrite(10,0);
   digitalWrite(11,1);
   digitalWrite(12,0);
}*/




void read_sensor()
{
 
  sensor[0]=digitalRead(12);//s2
  sensor[1]=digitalRead(A0);//s3
  sensor[2]=digitalRead(A1);//s4
  sensor[3]=digitalRead(A2);//s5
  sensor[4]=digitalRead(A3);//s6
  
  sensor[5]=digitalRead(A4);//s7
  sensor[6]=digitalRead(A5);//s8
  //sensor[7]=digitalRead(7);

   sensor1[0]=digitalRead(12);//s2
  sensor1[1]=digitalRead(A0);//s3
  sensor1[2]=digitalRead(A1);//s4
  sensor1[3]=digitalRead(A2);//s5
  sensor1[4]=digitalRead(A3);//s6
  
  sensor1[5]=digitalRead(A4);//s7
  sensor1[6]=digitalRead(A5);//s8
 
}




 
/*void checkpoint()
{
  read_sensor();
  if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0)&&(sensor[5]==0)&&(sensor[6]==0)&&(sensor[7]==0))
 {
  
  //digitalWrite(led,1);
delay(1000);
//digitalWrite(led,0);
}
  else if((sensor[0]==1)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0)&&(sensor[5]==0)&&(sensor[6]==0)&&(sensor[7]==1))
   {
  
 // digitalWrite(led,1);
delay(1000);
//digitalWrite(led,0);
}


  else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0)&&(sensor[5]==0)&&(sensor[6]==1)&&(sensor[7]==1))
   {
  
  //digitalWrite(led,1);
delay(1000);
//digitalWrite(led,0);
}

}
*/

void extra()
{read_sensor();
 
 if((sensor[0]==0)&&(sensor[6]==1))
 { 
  value=0;}
 else   if(( sensor[6]==0)&&(sensor[0]==1))
 { 
   value=1;}
  else 
  return;
}



void blank()
{
  //read_sensor();
  
   if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==1)&&(sensor[5]==1)&&(sensor[6]==1))
{
  if(value==1)
 {  error=8;}
 else
 error=-8;}
 else return;
}



void loop()
{
 // if((sensor[0]==sensor1[0])&&(sensor[1]==sensor1[1])&&(sensor[2]==sensor1[2])&&(sensor[3]==sensor1[3])&&(sensor[4]==sensor1[4])&&(sensor[5]==sensor1[5])&&(sensor[6]==sensor1[6])&&(sensor[7]==1))
   extra();
  //sensorReading();
 blank();
   
//sensorReading();
//checkpoint();
blackline();

calculate_pid();
 motor_control();
    
 
}

void sensorReading()
{
  Serial.print("lm---");
Serial.println(sensor[0]);
Serial.print("l---");
Serial.println(sensor[1]);
Serial.print("center---");
Serial.println(sensor[2]);
Serial.print("r---");
Serial.println(sensor[3]);
Serial.print("rm---");
Serial.println(sensor[4]);
Serial.print("clip----");
Serial.println(sensor[5]);
Serial.print("near----");
Serial.println(sensor[6]);
//delay(1000);
}








/*
void whiteline()
{

//read_sensor();
 
 
  if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==1)&&(sensor[4]==1)&&(sensor[5]==0)&&(sensor[6]==0)&&(sensor[7]==0))
  error=0;
 
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==1)&&(sensor[5]==0)&&(sensor[6]==0)&&(sensor[7]==0))
 { error=0; 

 
 }
else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==0)&&(sensor[5]==0)&&(sensor[6]==0)&&(sensor[7]==0))
  error=-7.4;
  

  
  
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==0)&&(sensor[4]==0)&&(sensor[5]==0)&&(sensor[6]==0)&&(sensor[7]==0))
  {error=-7.4;
 }
  
  
   else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==1)&&(sensor[5]==0)&&(sensor[6]==0)&&(sensor[7]==0))
  error=-7.4;
  
 
   else if((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==1)&&(sensor[5]==0)&&(sensor[6]==0)&&(sensor[7]==0))
  error=-7.4;
  else if((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==1)&&(sensor[5]==1)&&(sensor[6]==0)&&(sensor[7]==0))
  error=-7.4;
  
 
  //else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==1)&&(sensor[5]==1)&&(sensor[6]==0)&&(sensor[7]==0))
 
  
  
  
  else if((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==0)&&(sensor[4]==0)&&(sensor[5]==0)&&(sensor[6]==0)&&(sensor[7]==0))
  error=-7.4;
  
   else if((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0)&&(sensor[5]==0)&&(sensor[6]==0)&&(sensor[7]==0))
  error=-7.6;
  
 else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0)&&(sensor[5]==0)&&(sensor[6]==0)&&(sensor[7]==0))
  error=-7.7;
  
   else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==0)&&(sensor[4]==0)&&(sensor[5]==0)&&(sensor[6]==0)&&(sensor[7]==0))
  error=-7.7;
  
    
   else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==0)&&(sensor[5]==0)&&(sensor[6]==0)&&(sensor[7]==0))
  error=-8;
  
  
   else if((sensor[0]==1)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0)&&(sensor[5]==0)&&(sensor[6]==0)&&(sensor[7]==0))
  error=-7.7;
  
  
  else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==1)&&(sensor[5]==0)&&(sensor[6]==0)&&(sensor[7]==0))
  error=-8;
  
    else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==1)&&(sensor[5]==1)&&(sensor[6]==0)&&(sensor[7]==0))
  error=-8;
  
   
    else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==1)&&(sensor[5]==1)&&(sensor[6]==1)&&(sensor[7]==0))
  error=-8;
  
  
  //else if((sensor[0]==1)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==1)&&(sensor[4]==1)&&(sensor[5]==0)&&(sensor[6]==0)&&(sensor[7]==0))
    // { error=-8;
     //  forward_motor();
     //delay(300);
      //}
      
       else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==1)&&(sensor[5]==1)&&(sensor[6]==1)&&(sensor[7]==1))
      {error=8;
      
      }
      
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==1)&&(sensor[5]==1)&&(sensor[6]==0)&&(sensor[7]==0))
  error=7.4;
  
   else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0)&&(sensor[5]==1)&&(sensor[6]==0)&&(sensor[7]==0))
  error=7.4;
 
  
  
   else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0)&&(sensor[5]==0)&&(sensor[6]==1)&&(sensor[7]==0))
  error=7.5;
  
    else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0)&&(sensor[5]==0)&&(sensor[6]==0)&&(sensor[7]==1))
  error=8;
  
 else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==1)&&(sensor[4]==1)&&(sensor[5]==1)&&(sensor[6]==0)&&(sensor[7]==0))
  error=7.4;
  
   else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==1)&&(sensor[5]==1)&&(sensor[6]==1)&&(sensor[7]==0))
  error=7.4;
  
    else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0)&&(sensor[5]==1)&&(sensor[6]==1)&&(sensor[7]==0))
  error=7.5;
  
   else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0)&&(sensor[5]==1)&&(sensor[6]==1)&&(sensor[7]==1))
  error=8;
  
   else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==1)&&(sensor[5]==1)&&(sensor[6]==1)&&(sensor[7]==0))
  error=8;
  
  
  
 else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==1)&&(sensor[4]==1)&&(sensor[5]==1)&&(sensor[6]==1)&&(sensor[7]==0))
  error=7.8;


 else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0)&&(sensor[5]==0)&&(sensor[6]==1)&&(sensor[7]==1))
  error=7.9;
  
  
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0)&&(sensor[5]==0)&&(sensor[6]==0)&&(sensor[7]==1))
      error=8;
      
      
      
      else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==1)&&(sensor[4]==1)&&(sensor[5]==1)&&(sensor[6]==1)&&(sensor[7]==1))
      error=8;
      
      
       else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==1)&&(sensor[5]==1)&&(sensor[6]==1)&&(sensor[7]==1))
      error=8;
      
        else if((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==1)&&(sensor[5]==1)&&(sensor[6]==1)&&(sensor[7]==1))
      error=8;
      
      
    /*  else if((sensor[3]==1))
  error=0;
     else if((sensor[4]==1))
  error=0;*/
 
      
       //else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==1)&&(sensor[4]==1)&&(sensor[5]==0)&&(sensor[6]==0)&&(sensor[7]==1))
    // { forward_motor();
   // delay(250);
 // error=7;}
 /*else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0)&&(sensor[5]==0)&&(sensor[6]==0)&&(sensor[7]==0))
{
  if(value==1)
  error=8;
 else
 error=-8;}*/
 

void blackline()

 
{
  
  


  if((sensor[3]==0))
  error=0;

 else  if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==1)&&(sensor[5]==1)&&(sensor[6]==1))
  error=-3;
 else if((sensor[0]==1)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==1)&&(sensor[5]==1)&&(sensor[6]==1))
 error=-4;
 else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[3]==1)&&(sensor[4]==1)&&(sensor[5]==1)&&(sensor[6]==1))
 error=-6;
 else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==1)&&(sensor[5]==1)&&(sensor[6]==1))
 error=-8;
else if((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==1)&&(sensor[5]==1)&&(sensor[6]==1))
 error=-8;
 
 
 else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==0)&&(sensor[4]==0)&&(sensor[5]==1)&&(sensor[6]==1))
 error=3;
 
 else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==0)&&(sensor[5]==1)&&(sensor[6]==1))
 error=4;

  else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==0)&&(sensor[5]==0)&&(sensor[6]==1))
 error=6;


else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==1)&&(sensor[5]==1)&&(sensor[6]==0))
 error=8;

 }



void calculate_pid()
{
    P = error;
    I = I + previous_I;
    D = error-previous_error;
    
    PID_value = (Kp*P) + (Ki*I) + (Kd*D);
    
    previous_I=I;
    previous_error=error;
}

void motor_control()
{
    // Calculating the effective motor speed:
    int left_motor_speed = initial_motor_speed-PID_value;
    int right_motor_speed = initial_motor_speed+PID_value;
    
    // The motor speed should not exceed the max PWM value
    // constrain(left_motor_speed,0,255);
   // constrain(right_motor_speed,0,255);
       if(left_motor_speed >90 ) { left_motor_speed = 80.0; }
    if( left_motor_speed<-255.0 ) { left_motor_speed = -255.0; }
     if(right_motor_speed >90 ) { right_motor_speed = 80.0; }
    if( left_motor_speed<-255.0 ) { right_motor_speed = -255.0; }
  
  analogWrite(6,left_motor_speed);   //light Motor Speed new model
    analogWrite(10,right_motor_speed);  //Right Motor Speed new model
   // delay(10);
    //following lines of code are to make the bot move forward
    /*The pin numbers and high, low values might be different
    depending on your connections */
    //digitalWrite(4,HIGH);
    //digitalWrite(5,LOW);
    //digitalWrite(6,LOW);
    //digitalWrite(7,HIGH);
}


/*void blink() {
  state = !state;
  delay(30);
}*/



