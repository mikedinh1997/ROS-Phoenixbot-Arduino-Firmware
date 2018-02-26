#include <Servo.h>
#define ZEROPOINT 1500
#define DT 10000 //microseconds

//drive 7 & 11, unused 12-13 & 44-46
const char pwm[] = {2,3,4,5,6,7,11,12,13,44,45,46};
const char solenoid[] = {23,25,27,29,31,33};
const char analog[] = {A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11,A12,A13,A14,A15};
const char digital[] = {};
Servo servos[6];
//const char digital[30]; //todo: find out which DIO are free

//A1,B1,A2,B2
const char encoder[4] = {18,19,20,21};

char buffer[5];
volatile int64_t encoderCounts[] = {0,0};

//PID vars
float kp[] = {0.01,0};
float ki[] = {0,0};
float kd[] = {0,0};
int prv[] = {0,0}; //previous error from PID
int throttle[] = {0,0}; //counts per second; error correction after running PID
float ierr[] = {0,0}; //elapsed error
uint32_t runTime = 0; //how long since the program started
float err[] = {0,0}; 
float sp[] = {5500,0}; ///PID Set Point/////
char pid_flag[] = {0,0}; //set this after running an immediate PID so we have a reference point
uint32_t pid_time[] = {100000,100000}; //time elapsed since our last PID routine

void setup()
{
  Serial.begin(115200);
  delay(10);
  Serial.println("Beep boop, I am a robot.");

  for(char i = 0; i < 30; i++)
  {
  //  pinMode(digital[i], INPUT);
  }

  pinMode(solenoid[0], OUTPUT);
  pinMode(solenoid[1], OUTPUT);
  pinMode(solenoid[2], OUTPUT);

  attachInterrupt(digitalPinToInterrupt(encoder[0]), encoder1A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder[1]), encoder1B_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder[2]), encoder2A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder[3]), encoder2B_ISR, CHANGE);

  for(int i = 13; i > 6; i--)
  {
    servos[13-i].attach(i);
  
  }
  delay(10000); //10 second delay for synchronizing with Octave. Remove after PID loop is tuned.
}

void loop()
{
    char command;

    if((command = Serial.read()) >= 0)
    {
        while(Serial.available() <= 0);
        Serial.read(); //readout space
        int id;
        int value;
        char motor = 0;
        char pickSolenoid = 0;
        char solenoidState = 0;
        char pidMotor = 0;
        char pidInput = 0;
        char pidValue = 0;
        int speed;
        
        switch(command)
        {
          //command e.g. PID control - c pidInput pidValue (pidInput can be P, D, I, or S (setpoint))
            case 'A':
            case 'a':
              int pinNum;
              pinNum = (int)Serial.parseInt();
              Serial.read();//read \r
              Serial.println(analogRead(analog[pinNum]));
            case 'C':
            case 'c':
              while(Serial.available() <= 0);
              pidMotor = Serial.parseInt();
              Serial.read();
              pidInput = Serial.read();   
              pidValue = (pidInput == 's' || pidInput == 'S') ? Serial.parseFloat() :Serial.parseInt();

              if(pidInput == 'p' || pidInput == 'P')
              {
                 kp[pidMotor] = pidValue;
              
              }else if(pidInput == 'D' || pidInput == 'd')
              {
                kd[pidMotor] = pidValue;  
              
              }else if(pidInput == 'I' || pidInput == 'i')
              {
                 ki[pidMotor] = pidValue;
              
              }else if(pidInput == 'S' || pidInput == 's')
              {
                sp[pidMotor] =  pidValue;
              }
            
            Serial.read();
            break; 
           
            //pwm
            case 'P':
            case 'p':
              id = (int)Serial.parseInt();
              value = (int)Serial.parseInt();
              analogWrite(pwm[id], value);
              Serial.read(); //Read out extra \r
            break;
           
            //encoder: -1 reads both encoder values
            case 'E':
            case 'e':
               if(Serial.parseInt() == -1)
               {
                 itoa(encoderCounts[0],buffer,10);
                 buffer_Flush(buffer,16);
                 itoa(encoderCounts[1],buffer,10);
                 buffer_Flush(buffer,16);
                 
               }else if(Serial.parseInt() == 0)
               {
                 itoa(encoderCounts[0],buffer,10);
                 buffer_Flush(buffer,16);
                
               }else if(Serial.parseInt() == 1)
               {
                 itoa(encoderCounts[1],buffer,10);
                 buffer_Flush(buffer,16);
               }
               
               Serial.read(); // eats the char return /r
               
              break;

            //solenoid
              case 'S':
              case 's':
                while(Serial.available() <=0);
                pickSolenoid = Serial.parseInt();
                solenoidState  = Serial.parseInt();
                digitalWrite(solenoid[pickSolenoid],solenoidState);
                Serial.read();
              break;

            //digital read
              case 'D':
              case 'd':
                int pinNumber;
                pinNumber = (int)Serial.parseInt();
                Serial.read();//read \r
                Serial.println(digitalRead(digital[pinNumber]));
              break;

            //motor
              case 'M':
              case 'm':

              while(Serial.available() <= 0);
              motor = Serial.read();
              speed = (int)Serial.parseInt();
              
                if(motor == 'l' || motor == 'L')
                {
                  servos[0].writeMicroseconds(ZEROPOINT + speed);
                }
                else if(motor == 'r' || motor == 'R')
                {
                  servos[2].writeMicroseconds(ZEROPOINT + speed);
                }
                else
                {
                  Serial.println("Error with motor input");
                }
               Serial.read(); //Read out extra \r
               break;
               
            default:
                Serial.println("Error, serial input incorrect  ");
        }
    }
    //TODO: Set up timer to interrupt every 10ms and determine velocity. Alternatively we can do this through the PID loop 
    pid0();
    pid1();
}

void encoder1A_ISR()
{
  noInterrupts();
  if(digitalRead(encoder[0]) == HIGH)
  {
    if(digitalRead(encoder[1]) == LOW)
    {
      encoderCounts[0]++; //clockwise
    }
    else
    {
      encoderCounts[0]--; //counterclockwise
    }
  }
  else
  {
    if(digitalRead(encoder[1]) == HIGH)
    {
      encoderCounts[0]++; //clockwise
    }
    else
    {
      encoderCounts[0]--; //counterclockwise
    }
  }
  interrupts();
}

void encoder1B_ISR()
{
  noInterrupts();
  if(digitalRead(encoder[1]) == HIGH)
  {
    if(digitalRead(encoder[0]) == HIGH)
    {
      encoderCounts[0]++; //clockwise
    }
    else
    {
      encoderCounts[0]--; //counterclockwise
    }
  }
  else
  {
    if(digitalRead(encoder[0]) == LOW)
    {
      encoderCounts[0]++; //clockwise
    }
    else
    {
      encoderCounts[0]--; //counterclockwise
    }
  }
  interrupts();
}

void encoder2A_ISR()
{
  noInterrupts();
   if(digitalRead(encoder[2]) == HIGH)
  {
    if(digitalRead(encoder[3]) == LOW)
    {
      encoderCounts[1]++;
    }
    else
    {
      encoderCounts[1]--;
    }
  }
  else
  {
    if(digitalRead(encoder[3]) == HIGH)
    {
      encoderCounts[1]++;
    }
    else
    {
      encoderCounts[1]--;
    }
  }
  interrupts();
}

void encoder2B_ISR()
{
  noInterrupts();
  if(digitalRead(encoder[3]) == HIGH)
  {
    if(digitalRead(encoder[2]) == HIGH)
    {
      encoderCounts[1]++;
    }
    else
    {
      encoderCounts[1]--;
    }
  }
  else
  {
    if(digitalRead(encoder[2]) == LOW)
    {
      encoderCounts[1]++;
    }
    else
    {
      encoderCounts[1]--;
    }
  }
  interrupts();
}

void buffer_Flush(char *ptr, int length)
{
  for(int i = 0; i < length; i++)
  {
    ptr[i] = 0;
  }
}

void pid0()
{
  float t;

  if(!pid_flag[0])
  {
    err[0] = encoderCounts[0];
    pid_flag[0] = 1;
    pid_time[0] = micros();
    
  }
  
  if((micros() - pid_time[0]) >= DT)
  {
    t = ((float)(micros() - pid_time[0]))/1000000.0; //convert t to seconds
    //Serial.print(t);
    //Serial.print(" ");
    err[0] = encoderCounts[0] - err[0];
    err[0] /= t;
    byte* bytes = (byte*) &err[0];
    Serial.write(bytes,4);
    err[0] = sp[0] - err[0];
    
    //Serial.print(" ");
    //Serial.println(sp[0]);
    ierr[0] += err[0]*t;
    throttle[0] += kp[0] * err[0] + ki[0]*ierr[0] + kd[0]*((err[0] - prv[0])/t);
    if(throttle[0] >500)
    {
      throttle[0] = 500;
      
    }
     if(throttle[0] <-500)
    {
      throttle[0] = -500;
      
    }

    prv[0] = err[0];
  
    //Serial.print("Throttle: ");
    //Serial.println(throttle[0]);
    pid_flag[0] = 0;
    servos[0].writeMicroseconds(ZEROPOINT+throttle[0]);
  }
  //runTime = micros();
  
}
void pid1()
{
  float t;
  if(!pid_flag[1])
  {
    err[1] = encoderCounts[1];
    pid_flag[1] = 1;
    pid_time[1] = micros();
  }
  if(micros() - pid_time[1] >= DT)
  {
    t = ((float)(micros() - pid_time[1]))/1000000.0; //convert t to seconds
  
    err[1] = err[1] - encoderCounts[1];
    err[1] /= t;
    err[1] = sp[1] - err[1];
    byte* bytes = (byte*) &err[1];
    //Serial.write(bytes,4);
    ierr[1] += err[1]*t;
    throttle[1] += kp[1] * err[1] + ki[1]*ierr[1] + kd[1]*((err[1] - prv[1])/t);
    prv[1] = err[1];
    pid_flag[1] = 0;

    if(throttle[1] > 500)
    {
      throttle[1] = 500;
    }
    if(throttle[1] < -500)
    {
      throttle[1] = -500;
    }
    servos[1].writeMicroseconds(ZEROPOINT+throttle[1]);
  }
  //runTime = micros();
   
  
}


