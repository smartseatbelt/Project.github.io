#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX, TX

int potpin = 0;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin 

#define trigPin1 5 //setting the trig and echo pin of ultrasonic sensors
#define echoPin1 4
#define trigPin2 3
#define echoPin2 2

int lastSleepStatus = 0; // 1 = sleep ; 0 = drive
long distance; // Duration used to calculate distance
const int sensorvalue0 = A0;    // pin that the sensor is attached to 
const int sensorvalue1 = A1;    // pin that the sensor is attached to

int thresholdp0=127; // setting threashold for flex sensor connected to A0
int thresholdp1=440; // setting threashold for flex sensor connected to A1
int thresholds=60;   // setting threashold for potentiometer

void setup()
     {
  Serial.begin(9600);
  pinMode(trigPin1, OUTPUT);  // setting the input and output pins of ultrasonic sensors
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  mySerial.begin(9600);
     }

void loop() 
     {
  int sensorValue1 = analogRead(A1); // Reading the sensor value to aurduino
  int sensorValue0 = analogRead(A0); 

  Serial.print("a "); //print the sensor1 value in the serial monitor
  Serial.print(sensorValue0);
  Serial.print(" b ");  //print the ssensor2 value in the serial monitor
  Serial.print(sensorValue1);
  Serial.println(" ");
  
  val = analogRead(A2);            // reads the value of the potentiometer (value between 0 and 1023) 
  val = map(val, 0, 1023, 0, 240);     // scale it to use it with the servo (value between 0 and 180) 

  Serial.println("speed"); //print the speed value in the serial monitor
  Serial.println(val);

  digitalWrite(trigPin1, LOW);  //setting low and high values of ultrasonic sensor
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);

  long distance1 = pulseIn(echoPin1, HIGH); //....................................

  distance1 = distance1*0.034/2; //distance calculation 
  
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);

  long distance2 = pulseIn(echoPin2, HIGH);

  distance2 = distance2*0.034/2;
  
  Serial.print("Distance2:"); //print the distance value
  Serial.print(distance2);
  Serial.println();

    if((sensorValue0<=thresholdp0)||(sensorValue1<=thresholdp1)) //
       {
         if(distance1<30||distance2<30) 
            { 
               if (lastSleepStatus == 0) 
                   {
                      lastSleepStatus = 1;
                      mySerial.print("*#sleep:");
                      mySerial.print(val);
                      mySerial.print("#*!");
                      Serial.print("*#sleep:");
                      Serial.print(val);
                      Serial.println("#*!");
                   }


            }
         else
            {
            }
       }
    else 
       {
           if(val>thresholds)
               {
                  if ((sensorValue0>=thresholdp0)||(sensorValue1>=thresholdp1)) 
                      {          
                         if (lastSleepStatus == 1)
                             {
                               lastSleepStatus = 0;
                               mySerial.print("*#drive#*!");
                               Serial.print("*#drive#*!");
                             }

                      }
                  else  
                        {
                           if (lastSleepStatus == 0) 
                                  {
                                     lastSleepStatus = 1;
                                     Serial.print("*#sleep:");  
                                     Serial.print(val);
                                     Serial.println("#*!");
                                     mySerial.print("*#sleep:");  
                                     mySerial.print(val);
                                     mySerial.print("#*!");
                                  }
                        }
               }
  }  

  delay(1000); 
}








