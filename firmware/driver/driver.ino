#include <Servo.h>
#define ZEROPOINT 1500
/*
 * STEP <id> <steps> <step_rate>
 * PWM <id> <value>
 * ENCODER <id>
 * SOLENOID <id> <value>
 * DIGITAL <id>
 */

//steppers 2-6, drive 7 & 11, unused 12-13 & 44-46
const char pwm[] = {2,3,4,5,6,7,11,12,13,44,45,46};
const char solenoid[] = {8,9,10};
//const char digital[30]; //todo: find out which DIO are free

//A1,B1,A2,B2
const char encoder[4] = {18,19,20,21};

char buffer[16];
volatile long encoder1Count = 0;
volatile long encoder2Count = 0;
Servo leftMotor;
Servo rightMotor;

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

  attachInterrupt(5, encoder1A_ISR, CHANGE);
  attachInterrupt(4, encoder1B_ISR, CHANGE);
  attachInterrupt(3, encoder2A_ISR, CHANGE);
  attachInterrupt(2, encoder2B_ISR, CHANGE);

  leftMotor.attach(4);
  rightMotor.attach(5);
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
        int speed;
        switch(command)
        {
            //step
           case 'T':
            case 't':
                break;

            //pwm
            case 'P':
            case 'p':
              //while(Serial.available() <= 0);

              //Serial.readBytesUntil(' ', buffer, 16);

              //id = atoi(buffer);
              id = (int)Serial.parseInt();
              Serial.println(id);

             // buffer_Flush(buffer, 16);

              //while(Serial.available() <= 0);

              //Serial.readBytesUntil('\r',buffer,16);
              value = (int)Serial.parseInt();
              //value = atoi(buffer);
              Serial.println(value);

              //buffer_Flush(buffer, 16);

              analogWrite(pwm[id], value);
              //Serial.flush();
              Serial.read(); //Read out extra \r
                break;

            //enccoder
            case 'E':
            case 'e':
               Serial.print("Encoder 1: ");
               Serial.println(encoder1Count);
               Serial.print("Encoder 2: ");
               Serial.println(encoder2Count);
               Serial.read(); // eats the char return /r
                break;

            //solenoid
           case 'S':
            case 's':
                break;

            //digital read
            case 'D':
            case 'd':
                break;

            //motor
            case 'M':
            case 'm':

              while(Serial.available() <= 0);
               motor = Serial.read();
               //Serial.print("motor: ");
               //Serial.println(Serial.read());
                speed = (int)Serial.parseInt();

                Serial.print("motor: ");
                Serial.println(motor);
                Serial.println("speed: ");
                Serial.println(speed);

                if(motor == 'l' || motor == 'L')
                {
                  leftMotor.writeMicroseconds(ZEROPOINT + speed);
                }
                else if(motor == 'r' || motor == 'R')
                {
                  rightMotor.writeMicroseconds(ZEROPOINT + speed);
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
}

void encoder1A_ISR()
{
  if(digitalRead(encoder[0] == HIGH))
  {
    if(digitalRead(encoder[1] == LOW))
    {
      encoder1Count++;
    }
    else
    {
      encoder1Count--;
    }
  }
  else
  {
    if(digitalRead(encoder[1] == HIGH))
    {
      encoder1Count++;
    }
    else
    {
      encoder1Count--;
    }
  }

}

void encoder1B_ISR()
{
  if(digitalRead(encoder[1] == HIGH))
  {
    if(digitalRead(encoder[0] == HIGH))
    {
      encoder1Count++;
    }
    else
    {
      encoder1Count--;
    }
  }
  else
  {
    if(digitalRead(encoder[0] == LOW))
    {
      encoder1Count++;
    }
    else
    {
      encoder1Count--;
    }
  }

}

void encoder2A_ISR()
{
   if(digitalRead(encoder[2] == HIGH))
  {
    if(digitalRead(encoder[3] == LOW))
    {
      encoder2Count++;
    }
    else
    {
      encoder2Count--;
    }
  }
  else
  {
    if(digitalRead(encoder[3] == HIGH))
    {
      encoder2Count++;
    }
    else
    {
      encoder2Count--;
    }
  }
}

void encoder2B_ISR()
{
  if(digitalRead(encoder[3] == HIGH))
  {
    if(digitalRead(encoder[2] == HIGH))
    {
      encoder2Count++;
    }
    else
    {
      encoder2Count--;
    }
  }
  else
  {
    if(digitalRead(encoder[2] == LOW))
    {
      encoder2Count++;
    }
    else
    {
      encoder2Count--;
    }
  }

}

void buffer_Flush(char *ptr, int length)
{
  for(int i = 0; i < length; i++)
  {
    ptr[i] = 0;
  }
}
