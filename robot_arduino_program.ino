/** 
 * @file
 * @author Ruchir Tullu
 * @author Elizabeth Pratt
 * @author Hanya Wahdan
 * 
 * @defgroup Arduino_I2C_PIC
 * @ingroup Demo
 * @brief Demonstration of Arduino-PIC interfacing via I2C. In this program,
 *        the PIC outputs keypad data to the Arduino, which forwards it to a
 *        PC. If a triple-A sequence is given on the keypad, then the PIC will
 *        display data entered into the serial monitor on the character LCD. To
 *        open the serial monitor, go to Tools -> Serial Monitor
 * 
 * Preconditions:
 * @pre PIC-Arduino link switches are enabled (ON) for A4 and A5 of the Arduino
 *      Nano (RC4 and RC3 for the primary PIC, respectively)
 * @pre PIC and Arduino Nano agree on the Arduino Nano's slave address (i.e. the
 *      same slave address is used in software)
 * @pre A PC is connected if the Arduino Nano serial monitor is to be used. Note that
 *      the serial monitor baud rate must be 9600 for this program
 */

#include <Wire.h>

//connect gp2d120x to A1
#define pin A1  //IR sensor1

#define LEDPIN1 13
#define LEDPIN2 11    //set high if break beam broken --> on robot
#define SENSORPIN 12   //breakbeam
#define SENSORPIN2 7   //breakbeam
#define SENSORPIN3 8   //breakbeam
//#include "ultrasonic_2.c"
int sensorState = 0, lastState=0;         // variable for reading the pushbutton status   breakbeam
int sensorState2 = 0, lastState2=0;    
int sensorState3 = 0, lastState3=0;    

  //////////////// Break bream stuff /////////////////////////
  int encoder_pin = 2;  // The pin the encoder is connected           
  unsigned int rpm;     // rpm reading
  volatile byte pulses;  // number of pulses
  unsigned long timeold; 
  float rotations;
  long int distance;    //total distance
  // The number of pulses per revolution
  // depends on your index disc!!
  unsigned int pulsesperturn = 20;
//////////////////////////////////////////////////////////////

    //sensors going from lowest to highest (base --> top sensor)
    const int trigPin1 = 9;
    const int echoPin1 = 10;
    const int trigPin2 = 5;
    const int echoPin2 = 6;
    const int trigPin3 = 3;
    const int echoPin3 = 4;
    const int Ard_pic_comms_pin = 0;  //D0 on arduino and rc7 on pic

    //copy of receiveEvent variable:
    static uint8_t buf_copy[1] = {0};
        
    // defines variables
    long duration1;
    long duration2;
    long duration3;
    float distance1;
    float distance2;
    float distance3;
    float IR_distance;
    float dist1;
    float dist2;
    float dist3;
    // int mySensVals[5] = {1, 2, 3, 4}; //ultrasonic sensors 1 to 4, from bottom to top
    int base_radius = 4; //centimeters, change if incorrect

   //Flags for recieveEvent:
  volatile bool send_to_pic1 = false;
  volatile bool send_to_pic2 = false;
  volatile bool send_to_pic3 = false;
  volatile bool send_to_pic4 = false;
  volatile bool send_to_pic5 = false;
  
volatile bool send_to_pic = false;
volatile uint8_t incomingByte;    //uint8_t is the same as a byte. its shorthand for: a type of unsigned integer of length 8 bits   

//void recieveEvent();
//void requestEvent();

  bool is_tire1_there = false;
  bool is_tire2_there = false;

  int result;
  int clarified_result = 99;    //random test value
  int x;
  int flag2 = 5; //random value

  
// break beam function
 void counter()
 {
    //Update count
      pulses++;    
      if (pulses == 2){
        rotations += 0.1/2;
      }
 }


//////////////////////////////// FUNCTION PROTOTYPES ////////////////////////////
int tire_exist(float data1, float data2, float data3);      //function declaration
void requestEvent(void);
void receiveEvent(void);
float read_ultrasonic_sensor1();
float read_ultrasonic_sensor2();
float read_ultrasonic_sensor3();
/////////////////////////////////////////////////////////////////////////////////


void setup(){
    // defines pins numbers
        pinMode(pin, INPUT);    //IR sensor1
    
  ////////////shaft encoders/////////////////////////////
    //Use statusPin to flash along with interrupts
   pinMode(encoder_pin, INPUT);
   pinMode(Ard_pic_comms_pin, OUTPUT);    //arduino pin to pic pin

      //Interrupt 0 is digital pin 2, so that is where the IR detector is connected
   //Triggers on FALLING (change from HIGH to LOW)
   attachInterrupt(0, counter, FALLING);
   // Initialize
   pulses = 0;
   rpm = 0;
   timeold = 0;
//////////////////////////////////////////////////////////
    
  ////////////break beam/////////////////////////////       //theres 3 of the break beams now
  // initialize the LED pin as an output:
  pinMode(LEDPIN1, OUTPUT);      
  pinMode(LEDPIN2, OUTPUT);   
  // initialize the sensor pin as an input:
  pinMode(SENSORPIN, INPUT);     
  digitalWrite(SENSORPIN, HIGH); // turn on the pullup
    pinMode(SENSORPIN2, INPUT);     
  digitalWrite(SENSORPIN2, HIGH); // turn on the pullup
    pinMode(SENSORPIN3, INPUT);     
  digitalWrite(SENSORPIN3, HIGH); // turn on the pullup
  /////////////////////////////////////////////////////

    pinMode(trigPin1, OUTPUT); // Sets the trigPin as an Output
    pinMode(echoPin1, INPUT); // Sets the echoPin as an Input
    pinMode(trigPin2, OUTPUT); // Sets the trigPin as an Output
    pinMode(echoPin2, INPUT); // Sets the echoPin as an Input
    pinMode(trigPin3, OUTPUT); // Sets the trigPin as an Output
    pinMode(echoPin3, INPUT); // Sets the echoPin as an Input
    //Serial.begin(9600); // Starts the serial communication
    
    Wire.begin(8); // Join I2C bus with address 8
  
    // Register callback functions
    Wire.onReceive(receiveEvent); // Called when this slave device receives a data transmission from master
    Wire.onRequest(requestEvent); // Called when master requests data from this slave device
  
    // Open serial port to PC (hardware UART)
    Serial.begin(9600);      
}



void loop(){    //runs repeatedly

  while(1){
//
//              ////////////////////////////// Shaft encoder distance/////////////////////////////////////////////////
//              if (millis() - timeold >= 1000){  /*Uptade every one second, this will be equal to reading frecuency (Hz).*/
//               
//                //Don't process interrupts during calculations
//                 detachInterrupt(0);
//                 //Note that this would be 60*1000/(millis() - timeold)*pulses if the interrupt
//                 //happened once per revolution
//                 rpm = (60 * 1000 / pulsesperturn )/ (millis() - timeold)* pulses;
//                 timeold = millis();
//                 pulses = 0;
////                 distance = (20.9 * rotations);
//                 //Write it out to serial port
//                 Serial.print("RPM = ");
//                 Serial.println(rpm,DEC);
//                 Serial.print("Distance =  ");
//                 Serial.println(distance,DEC);
//                 //Restart the interrupt processing
//                 attachInterrupt(0, counter, FALLING);
//              }
//            ///////////////////////////////////////////////////////////////////////////////////////////////////////////


        // read the state of the break beam value:
        sensorState = digitalRead(SENSORPIN);
        sensorState2 = digitalRead(SENSORPIN2);
        sensorState3 = digitalRead(SENSORPIN3);
        
        // If we should send to the PIC, then we wait to receive a byte from the PIC
        if (send_to_pic1 == true && !incomingByte){
           float dist_accum1 = 0;
           int counter1 = 0;
          for(int i = 0; i<5; i++){
             // Clears the trigPin
                  digitalWrite(trigPin1, LOW);
                  delayMicroseconds(2);
                  // Sets the trigPin on HIGH state for 10 micro seconds
                  digitalWrite(trigPin1, HIGH);
                  delayMicroseconds(10);
                  digitalWrite(trigPin1, LOW);
                  // Reads the echoPin, returns the sound wave travel time in microseconds
                  duration1 = pulseIn(echoPin1, HIGH);
                  // Calculating the distance
                  dist1= duration1*0.034/2.0;
                  if (dist1 > 0){
                    dist_accum1 +=dist1;
                    counter1 += 1;
                  }
          }
          dist_accum1 = dist_accum1/counter1;

             distance1 = dist_accum1;
             //pulseIn(echoPin2, LOW);
             
              //Sensor 1:
              //distance1 = 101;
             //distance1 = read_ultrasonic_sensor1();
             x = distance1;
             Serial.println("Distance1:");
             Serial.println(distance1);

        }
  

        else if (send_to_pic2 == true && !incomingByte) {   //&& Serial.available() 
              Serial.println("loop2");

              //clarified_result = 45;

              //Sensor 1:
            // distance1 = read_ultrasonic_sensor1();     //already has 
            //delay(50);
              //Sensor 2
          float dist_accum2 = 0;
          for(int j = 0; j<5; j++){
             // Clears the trigPin
                  digitalWrite(trigPin2, LOW);
                  delayMicroseconds(2);
                  // Sets the trigPin on HIGH state for 10 micro seconds
                  digitalWrite(trigPin2, HIGH);
                  delayMicroseconds(10);
                  digitalWrite(trigPin2, LOW);
                  // Reads the echoPin, returns the sound wave travel time in microseconds
                  duration2 = pulseIn(echoPin2, HIGH);
                  // Calculating the distance
                  dist2= duration2*0.034/2.0;
                  dist_accum2 +=dist2;
          }
          dist_accum2 = dist_accum2/5;
          distance2 = dist_accum2;
          //pulseIn(echoPin2, LOW);
             //delay(50);
              //Sensor 3
          
          
          
          float dist_accum3 = 0;
          for(int k = 0; k<5; k++){
             // Clears the trigPin
                  digitalWrite(trigPin3, LOW);
                  delayMicroseconds(2);
                  // Sets the trigPin on HIGH state for 10 micro seconds
                  digitalWrite(trigPin3, HIGH);
                  delayMicroseconds(10);
                  digitalWrite(trigPin3, LOW);
                  // Reads the echoPin, returns the sound wave travel time in microseconds
                  duration3 = pulseIn(echoPin3, HIGH);
                  // Calculating the distance
                  dist3= duration3*0.034/2.0;
                  dist_accum3 +=dist3;
          }
          dist_accum3 = dist_accum3/5;
          distance3 = dist_accum3;
       //   pulseIn(echoPin3, LOW);

//////////////////////////////////////////////// result function/////////////////////////////////////////////////////////
            int tires_there_accum = 0;
      if ((distance2 - distance1) <= 2.5){     //3.2 , 1
            int tire_there1 = 1;
            tires_there_accum += tire_there1;
      }
      if ((( distance3) - distance1) <= 2.5){     //3.2 , 1    //so now sensor 3 is on the other side of the wall, hence this: (20 - data3 -2) mimics the distance as it was before on the other wall.
            int tire_there2 = 1;
            tires_there_accum += tire_there2;
      }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                    clarified_result = tires_there_accum; //return val
                    //clarified_result = 39;
                    flag2 = 1;    //set flag high if done computations

              //result = tire_exist(distance1, distance2, distance3); //returns either a 0, 1, or 2 depending on how many tires there are on the pole
              //clarified_result = result;
 
              // Prints the distance on the Serial Monitor
             Serial.print("DIST1:");
             Serial.println(distance1, DEC);   
             Serial.print("DIST2:");
             Serial.println(distance2, DEC);    
             Serial.print("DIST3:");
             Serial.println(distance3, DEC); 
             Serial.print("Result:");
             Serial.println(clarified_result);   
             
//            uint16_t value = analogRead (pin);  
//            IR_distance = get_IR (value); //Convert the analog voltage to the distance
//            Serial.println (value);                 //Print the data to the arduino serial monitor
//            Serial.print (IR_distance);
//            Serial.println (" cm");
//            Serial.println ();
//            delay (300);                            //Delay 0.5s

        }

          else if (send_to_pic3 == true && !incomingByte) {
            //lastState2 = sensorState2;
//              Serial.println("loop3");
//              
//             //Sensor 3
//             distance3 =  read_ultrasonic_sensor3();
//
//             Serial.print("Distance3:");
//             Serial.println(distance3);
          int dist_accum4 = 0;
          for(int z = 0; z<5; z++){
             // Clears the trigPin
                  digitalWrite(trigPin3, LOW);
                  delayMicroseconds(2);
                  // Sets the trigPin on HIGH state for 10 micro seconds
                  digitalWrite(trigPin3, HIGH);
                  delayMicroseconds(10);
                  digitalWrite(trigPin3, LOW);
                  // Reads the echoPin, returns the sound wave travel time in microseconds
                  duration3 = pulseIn(echoPin3, HIGH);
                  // Calculating the distance
                  dist3= duration3*0.034/2.0;
                  dist_accum4 +=dist3;
          }
          dist_accum4 = dist_accum4/5;
          distance3 = dist_accum4;

          }

         else if (send_to_pic4 == true && !incomingByte) {    //break beam

              // check if the sensor beam is broken
              // if it is, the sensorState is LOW:
              if (sensorState == LOW) {     
                // turn LED on:
                digitalWrite(LEDPIN1, HIGH);  
                digitalWrite(LEDPIN2, HIGH);  
              } 
              else {
                // turn LED off:
                digitalWrite(LEDPIN1, LOW); 
                digitalWrite(LEDPIN2, LOW);  
              }
              
              if (sensorState && !lastState) {
                Serial.println("Unbroken");
              } 
              if (!sensorState && lastState) {
                Serial.println("Broken");
              }
              lastState = sensorState;
              
          }


         else if (send_to_pic5 == true && !incomingByte) {    //shaft encoder
                distance = (20.9 * rotations);//circumference of tire * rotations
          }

      }

      
  }

//////////////////////////////////////////////////////////
///////////////////FUNCTIONS//////////////////////////////
//////////////////////////////////////////////////////////

//////////////////////setup functions/////////////////////
/** @brief Callback for when the master transmits data */
void receiveEvent(void){
    static uint8_t buf[1] = {0};
    //static uint8_t counter = 0;
    uint8_t x = Wire.read(); // Receive byte
    buf[0] = x;
    digitalWrite(Ard_pic_comms_pin, LOW);    //set to low if receiving something
    Serial.println("");
    Serial.println((char)x); // Print to serial output as char (ASCII representation)
    Serial.println("");
   // buf[counter++] = x;
    //counter = (counter == 3) ? 0 : counter;     //An expression a ? b : c evaluates to b if the value of a is true, and otherwise to c
    Serial.println("BUF0:");
    Serial.println(buf[0]);
//    Serial.println("BUF1:");
//    Serial.println(buf[1]);
//    Serial.println("BUF2:");
//    Serial.println(buf[2]);
    Serial.println("");
    
    if(buf[0]== 65){   //first sensor is called   =='A'
        Serial.println("A: hello");
        send_to_pic1 = true;
        send_to_pic2 = false;
        send_to_pic3 = false;
        send_to_pic4 = false;
        send_to_pic5 = false;
    }

    else if(buf[0]== 66 ){   //second sensor is called  //buf[0]|| buf[1] || buf[2]=='B'
      Serial.println("B: hello");
        send_to_pic1 = false;   //false
        send_to_pic2 = true;
        //Serial.println(send_to_pic2);
        send_to_pic3 = false;
        send_to_pic4 = false;
        send_to_pic5 = false;
    }

    else if(buf[0]==67 ){   //sensor 3
        Serial.println("C: hello");
        send_to_pic1 = false;   //false
        send_to_pic2 = false;
        send_to_pic3 = true;
        send_to_pic4 = false;
        send_to_pic5 = false;
    }

    else if(buf[0]==68 ){   //sensor 4 break beam
        Serial.println("D: hello");
        send_to_pic1 = false;   //false
        send_to_pic2 = false;
        send_to_pic3 = false;
        send_to_pic4 = true;
        send_to_pic5 = false;
    }

     else if(buf[0]==69 ){   //shaft encoder == distance
        Serial.println("E: hello");
        send_to_pic1 = false;   //false
        send_to_pic2 = false;
        send_to_pic3 = false;
        send_to_pic4 = false;
        send_to_pic5 = true;
    }

    buf_copy[0] = buf[0];
    //else{
     // send_to_pic1 = true;
    //}
}

/** @brief Callback for when the master requests data */
void requestEvent(void){
//    Serial.println("incomingbyte here");
//    Serial.println(incomingByte);
//    Serial.println("");

      if (buf_copy[0] == 'A'){    //ultrasonic 1
          //incomingByte = distance1 * 10;  //debug for multiply by 10 here and divide by 10 on pic to retain decimal point
          //wait then send a lower case a to signal the end of sending Incoming Byte
          //delay(500);
          //incomingByte = 96;
         incomingByte = (int) x;    //distance1
      }
      else if (buf_copy[0] == 'B'){    //ultrasonic 2

        
              //Sensor 1:
            // distance1 = read_ultrasonic_sensor1();     //already has this value
            //delay(50);
              //Sensor 2
          float dist_accum2 = 0;
          int counter2 = 0;
          for(int j = 0; j<15; j++){
             // Clears the trigPin
                  digitalWrite(trigPin2, LOW);
                  delayMicroseconds(2);
                  // Sets the trigPin on HIGH state for 10 micro seconds
                  digitalWrite(trigPin2, HIGH);
                  delayMicroseconds(10);
                  digitalWrite(trigPin2, LOW);
                  // Reads the echoPin, returns the sound wave travel time in microseconds
                  duration2 = pulseIn(echoPin2, HIGH);
                  // Calculating the distance
                  dist2= duration2*0.034/2.0;
                  if ((dist2 >0) && (dist2 < 22)){ 
                    dist_accum2 +=dist2;
                    counter2 +=1;
                  }
          }
          dist_accum2 = dist_accum2/counter2;
          distance2 = dist_accum2;
          pulseIn(echoPin2, LOW);
             //delay(50);
              //Sensor 3
          
          
          
          float dist_accum3 = 0;
          int counter3 = 0;
          for(int k = 0; k<15; k++){
             // Clears the trigPin
                  digitalWrite(trigPin3, LOW);
                  delayMicroseconds(2);
                  // Sets the trigPin on HIGH state for 10 micro seconds
                  digitalWrite(trigPin3, HIGH);
                  delayMicroseconds(10);
                  digitalWrite(trigPin3, LOW);
                  // Reads the echoPin, returns the sound wave travel time in microseconds
                  duration3 = pulseIn(echoPin3, HIGH);
                  // Calculating the distance
                  dist3= duration3*0.034/2.0;
                  if ((dist3 >0) && (dist3 < 22)){
                    dist_accum3 +=dist3;
                    counter3 +=1;
                  }
          }
          dist_accum3 = dist_accum3/counter3;
          distance3 = dist_accum3;
          pulseIn(echoPin3, LOW);

//////////////////////////////////////////////// result function/////////////////////////////////////////////////////////
            int tires_there_accum = 0;
      if ((distance2 - distance1) <= 2.5){     //3.2 , 1
            int tire_there1 = 1;
            tires_there_accum += tire_there1;
      }
      if (((distance3) - distance1) <= 2.5){     //3.2 , 1    //so now sensor 3 is on the other side of the wall, hence this: (20 - data3 -2) mimics the distance as it was before on the other wall.
            int tire_there2 = 1;
            tires_there_accum += tire_there2;
      }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                    clarified_result = tires_there_accum; //return val
                    //clarified_result = 39;
                    flag2 = 1;    //set flag high if done computations

                    
          //incomingByte = distance2 * 10;
            //incomingByte = IR_distance;
            //wait then send a lower case a to signal the end of sending Incoming Byte
         // delay(500);
         // incomingByte = (int) clarified_result;    //0, 1, or 2 tires on pole   //char or int or bitshift
         Serial.println("flag2");
         Serial.println(flag2);
         //while (flag2 != 1){
          incomingByte = clarified_result;
          Serial.println("clarifiedResult");
         Serial.println(clarified_result);
         Serial.println("DISTANCEONE");
         Serial.println(distance1);
         Serial.println("DISTANCETWO");
         Serial.println(distance2);
         Serial.println("DISTANCETHREE");
         Serial.println(distance3);
          //Serial.println("f2");
         //Serial.println(flag2);
         //}
         flag2 = 0; //set to zero right after
         Serial.println("flag2");
         Serial.println(flag2);
         //incomingByte = 49;
          //incomingByte = 1;

      }

      else if (buf_copy[0] == 'C'){        //ultrasonic 3
         //Send a 0 if not broken and send a 1 if broken
        if (sensorState2 == LOW){    // broken
           incomingByte = 200;
           Serial.println("it's broken lol");
           Serial.println(sensorState2);
        }
        else if (sensorState2 == HIGH){   //not broken
           incomingByte = 100;
           Serial.println("it ain't broken lol");
           Serial.println(sensorState2);
        }
        
          //incomingByte = (int) distance3;     //* 10;
          //wait then send a lower case a to signal the end of sending Incoming Byte
          //delay(500);
          //incomingByte = 116;
      }
      
      else if (buf_copy[0] == 'D'){    //break beam
        //Send a 0 if not broken and send a 1 if broken
        if (sensorState == LOW){    // broken
           incomingByte = 200;
        }
        else if (sensorState == HIGH){   //not broken
           incomingByte = 100;
        }
        
      }

      else if (buf_copy[0] == 'E'){    //shaft encoder
          incomingByte = distance;
        //if (distance == 255){   //if it exceeds the byte limit
          //distance = 0;   //clear distance right after
        //}
          //distance = 0;   //clear distance right after
      }
        Serial.println("");
    Serial.println("incomingbyte here");
   Serial.println(incomingByte);
    Serial.println("");

    digitalWrite(Ard_pic_comms_pin, HIGH);    //set to high if sending something
    Wire.write(incomingByte); // Respond with message of 1 byte
    delay(1000);    //delay of 1 second
    incomingByte = 0; // Clear output buffer
}
/////////////////////////////////////////////////////////////////

float read_ultrasonic_sensor1(){
            float dist1;
            float distance_accum1 = 0;
            int counter = 0;
                //Sensor 1:
  
              for (int i = 0; i<10; i++){    //loop pinging 5 times
                  // Clears the trigPin
                  digitalWrite(trigPin1, LOW);
                  delayMicroseconds(2);
                  // Sets the trigPin on HIGH state for 10 micro seconds
                  digitalWrite(trigPin1, HIGH);
                  delayMicroseconds(10);
                  digitalWrite(trigPin1, LOW);
                  // Reads the echoPin, returns the sound wave travel time in microseconds
                  duration1 = pulseIn(echoPin1, HIGH);
                  // Calculating the distance
                  dist1= duration1*0.034/2.0;
                  //distance1 = (int) distance1;    //convert to int
                  if(dist1!=0){
                     distance_accum1 += dist1;
                     counter+=1;
                  }
            }
            //digitalWrite(trigPin1, LOW);
           // pulseIn(echoPin1, LOW);
            //return distance_accum1 / counter;
            return 101.00;   //problematic
              
}




float read_ultrasonic_sensor2(){
            float dist2;
            float distance_accum2 = 0;
            int counter2 = 0;
            for (int i = 0; i <10; i++){
                  //Sensor 2
                  // Clears the trigPin
                  digitalWrite(trigPin2, LOW);
                  delayMicroseconds(2);
                  // Sets the trigPin on HIGH state for 10 micro seconds
                  digitalWrite(trigPin2, HIGH);
                  delayMicroseconds(10);
                  digitalWrite(trigPin2, LOW);
                  // Reads the echoPin, returns the sound wave travel time in microseconds
                  duration2 = pulseIn(echoPin2, HIGH);
                  // Calculating the distance
                  dist2= duration2*0.034/2.0;
                  //distance2 = (int) distance2;    //convert to int
                  if(dist2!=0){
                     distance_accum2 += dist2;
                     counter2 += 1;
                  }
            }
            //digitalWrite(trigPin2, LOW);
            pulseIn(echoPin2, LOW);
            return distance_accum2 / counter2;
}


float read_ultrasonic_sensor3(){
            float dist3;
            float distance_accum3 = 0;
            int counter3  = 0;
            for (int i = 0; i <10; i++){
                  //Sensor 3
                  // Clears the trigPin
                  digitalWrite(trigPin3, LOW);
                  delayMicroseconds(2);
                  // Sets the trigPin on HIGH state for 10 micro seconds
                  digitalWrite(trigPin3, HIGH);
                  delayMicroseconds(10);
                  digitalWrite(trigPin3, LOW);
                  // Reads the echoPin, returns the sound wave travel time in microseconds
                  duration3 = pulseIn(echoPin3, HIGH);
                  //Serial.println(duration3);
                  // Calculating the distance
                  dist3= duration3*0.034/2.0;
                  //distance3 = (int) distance3;    //convert to int
                  //return distance;
                  if(dist3!=0){
                      distance_accum3 += dist3;
                      counter3 +=1;
                  }
            }
            //digitalWrite(trigPin3, LOW);
            pulseIn(echoPin3, LOW);
            return distance_accum3 / counter3;
}


////return distance (cm)
//float get_IR (uint16_t value) {
//        if (value < 16)  value = 16;
//        return 2076.0 / (value - 11.0);
//}

int tire_exist(float data1, float data2, float data3){    //base sensor, sensor 2 and 3
  int tires_there_accum = 0;
      if ((data2 - data1) <= 2.5){     //3.2 , 1
            int tire_there1 = 1;
            tires_there_accum += tire_there1;
      }
      if (((20 - data3 -2) - data1) <= 2.5){     //3.2 , 1    //so now sensor 3 is on the other side of the wall, hence this: (20 - data3 -2) mimics the distance as it was before on the other wall.
            int tire_there2 = 1;
            tires_there_accum += tire_there2;
      }

      return tires_there_accum;
}

//////////////////////////////////////////////////////////
/////////////////// END FUNCTIONS////////////////////////
//////////////////////////////////////////////////////////










//////////////////////////////////////// IncomingByte old logic///////////////////////////
          //initialize variables
//          int difference_tire_1 = 0;
//          int difference_tire_2 = 0;
//  
//          
//          difference_tire_1 = distance2 - base_radius;    //if tire is there then this distance should be almost the same as the distance1
//          difference_tire_2 - distance3 - base_radius;    //if tire is there then this distance should be almost the same as the distance1
//          Serial.println("diff 1 = "); 
//          Serial.println(difference_tire_1);
//          Serial.println("diff 2 = "); 
//          Serial.println(difference_tire_2);
//  
//          //check if tire there and set flags accordingly
//          if ( (distance2 < 30) && (abs(difference_tire_1-distance1) < 1) ){    //check if less than 30 cm and if 
//            incomingByte = incomingByte || (0xF);    //load 1111 into lower 4 bits of 8-bit incomingByte; this will tell the pic that there is a tire in the lower slot of the base
//          //incomingByte = 5;
//                   Serial.println("Incoming Byte Test:");
//                   Serial.println(incomingByte);
//
//          }
//            
//          if ( (distance3 < 30) && (abs(difference_tire_2-distance1) < 1) ){    //check if less than 30 cm and if 
//            incomingByte = incomingByte || (0xF<<4);    //load 1111 into upper 4 bits of 8-bit incomingByte; this will tell the pic that there is a tire in the lower slot of the base
//                    //incomingByte = 6;
//          }
          //incomingByte = distance2;
//      }

      
 // }
///////////////////////////////////////////////////////////////////////////////////////

















///debug junkyard///

//for iterating through a loop

//int cnt = 0;
//while(cnt<5){
//      for (int i =0; i<5; i++){
//        Serial.println("Value of index:");
//        Serial.println(i);
//        Serial.println("mySensVals value:");
//        Serial.println(mySensVals[i]);
//        cnt+=1;
//        delay(500);
//      }
//}
//while(1){
//  Serial.println{"done");
//}





//////Previous version////////////


/** 
 * @file
 * @author Michael Ding
 * @author Tyler Gamvrelis
 * 
 * @defgroup Arduino_I2C_PIC
 * @ingroup Demo
 * @brief Demonstration of Arduino-PIC interfacing via I2C. In this program,
 *        the PIC outputs keypad data to the Arduino, which forwards it to a
 *        PC. If a triple-A sequence is given on the keypad, then the PIC will
 *        display data entered into the serial monitor on the character LCD. To
 *        open the serial monitor, go to Tools -> Serial Monitor
 * 
 * Preconditions:
 * @pre PIC-Arduino link switches are enabled (ON) for A4 and A5 of the Arduino
 *      Nano (RC4 and RC3 for the primary PIC, respectively)
 * @pre PIC and Arduino Nano agree on the Arduino Nano's slave address (i.e. the
 *      same slave address is used in software)
 * @pre A PC is connected if the Arduino Nano serial monitor is to be used. Note that
 *      the serial monitor baud rate must be 9600 for this program
 */
//
//#include <Wire.h>
////#include "ultrasonic_2.c"
//
//    //sensors going from lowest to highest (base --> top sensor)
//    const int trigPin1 = 9;
//    const int echoPin1 = 10;
//    const int trigPin2 = 4;
//    const int echoPin2 = 5;
//    const int trigPin3 = 2;
//    const int echoPin3 = 3;
//
//        
//    // defines variables
//    long duration1;
//    long duration2;
//    long duration3;
//    float distance1;
//    float distance2;
//    float distance3;
//    // int mySensVals[5] = {1, 2, 3, 4}; //ultrasonic sensors 1 to 4, from bottom to top
//    int base_radius = 4; //centimeters, change if incorrect
//
//   //Flags for recieveEvent:
//  volatile bool send_to_pic1 = false;
//  volatile bool send_to_pic2 = false;
//  volatile bool send_to_pic3 = false;
//
//volatile bool send_to_pic = false;
//volatile uint8_t incomingByte;    //uint8_t is the same as a byte. its shorthand for: a type of unsigned integer of length 8 bits   
//
////void recieveEvent();
////void requestEvent();
//
//  bool is_tire1_there = false;
//  bool is_tire2_there = false;
//
//
//
////////////////////////setup functions/////////////////////
///** @brief Callback for when the master transmits data */
//void receiveEvent(void){
//    static uint8_t buf[1] = {5};
//    //static uint8_t counter = 0;
//    
//    uint8_t x = Wire.read(); // Receive byte
//    buf[0] = x;
//    Serial.println((char)x); // Print to serial output as char (ASCII representation)
//    
//   // buf[counter++] = x;
//    //counter = (counter == 3) ? 0 : counter;     //An expression a ? b : c evaluates to b if the value of a is true, and otherwise to c
//    Serial.println("BUF0:");
//    Serial.println(buf[0]);
////    Serial.println("BUF1:");
////    Serial.println(buf[1]);
////    Serial.println("BUF2:");
////    Serial.println(buf[2]);
//    Serial.println("");
//    if(buf[0]== 65){   //first sensor is called   =='A'
//        Serial.println("A: hello");
//        send_to_pic1 = true;
//        send_to_pic2 = false;
//        send_to_pic3 = false;
//    }
//
//    if(buf[0]== 66 ){   //second sensor is called  //buf[0]|| buf[1] || buf[2]=='B'
//      Serial.println("B: hello");
//        send_to_pic1 = false;   //false
//        send_to_pic2 = true;
//         Serial.println(send_to_pic2);
//        send_to_pic3 = false;
//    }
//
//    if(buf[0]==67 ){   //sensor 3
//        Serial.println("C: hello");
//        send_to_pic1 = false;   //false
//        send_to_pic2 = false;
//        send_to_pic3 = true;
//    }
//
//    //else{
//     // send_to_pic1 = true;
//    //}
//}
//
///** @brief Callback for when the master requests data */
//void requestEvent(void){
//    Serial.println("incomingbyte here");
//    Serial.println(incomingByte);
//    Serial.println("");
//    Wire.write(incomingByte); // Respond with message of 1 byte
//    incomingByte = 0; // Clear output buffer
//}
///////////////////////////////////////////////////////////////////
//
//
//
//
//void setup(){
//    // defines pins numbers
//
//
//    pinMode(trigPin1, OUTPUT); // Sets the trigPin as an Output
//    pinMode(echoPin1, INPUT); // Sets the echoPin as an Input
//    pinMode(trigPin2, OUTPUT); // Sets the trigPin as an Output
//    pinMode(echoPin2, INPUT); // Sets the echoPin as an Input
//    pinMode(trigPin3, OUTPUT); // Sets the trigPin as an Output
//    pinMode(echoPin3, INPUT); // Sets the echoPin as an Input
//    Serial.begin(9600); // Starts the serial communication
//    
//    Wire.begin(8); // Join I2C bus with address 8
//  
//    // Register callback functions
//    Wire.onReceive(receiveEvent); // Called when this slave device receives a data transmission from master
//    Wire.onRequest(requestEvent); // Called when master requests data from this slave device
//  
//    // Open serial port to PC (hardware UART)
//    Serial.begin(9600);      
//}
//
//
//
//void loop(){    //runs repeatedly
//
//  while(1){
//        //send_to_pic = true;
//        //incomingByte = 0x00;
//        
//        // If we should send to the PIC, then we wait to receive a byte from the PIC
//        if (send_to_pic1 == true && !incomingByte){
//              //ping first sensor and write to incomingByte
//              //Sensor 1:
//  
//             // for (int i = 0; i<5; i++){    //loop pinging 5 times
//                  // Clears the trigPin
//                  digitalWrite(trigPin1, LOW);
//                  delayMicroseconds(2);
//                  // Sets the trigPin on HIGH state for 10 micro seconds
//                  digitalWrite(trigPin1, HIGH);
//                  delayMicroseconds(10);
//                  digitalWrite(trigPin1, LOW);
//                  // Reads the echoPin, returns the sound wave travel time in microseconds
//                  duration1 = pulseIn(echoPin1, HIGH);
//                  // Calculating the distance
//                  distance1= duration1*0.034/2.0;
//                  distance1 = (int) distance1;    //convert to int
//             // }
//              //set incomingByte
//              if ((distance1 > 0) && distance1 < 30){
//              incomingByte = distance1;
//             }
//              else{
//              incomingByte = 250;
//             }
//             Serial.println("distance1:");
//             Serial.println(distance1);
//             Serial.println("imcomingByte for distance1:");
//             Serial.println(incomingByte); //,DEC
//             //send_to_pic1 = false;   //debug
//        }
//  
//        //send_to_pic2 = true;
//        // If we should send to the PIC, then we wait to receive a byte from the PIC
//        //Serial.println(send_to_pic2);   //debug
//        if (send_to_pic2 == true && !incomingByte) {   //&& Serial.available() 
//              Serial.println("loop2");
//              //delay(500);
//            //for (int i = 0; i<5; i++){    //do five times
//                //Sensor 2
//                  // Clears the trigPin
//                  digitalWrite(trigPin2, LOW);
//                  delayMicroseconds(2);
//                  // Sets the trigPin on HIGH state for 10 micro seconds
//                  digitalWrite(trigPin2, HIGH);
//                  delayMicroseconds(10);
//                  digitalWrite(trigPin2, LOW);
//                  // Reads the echoPin, returns the sound wave travel time in microseconds
//                  duration2 = pulseIn(echoPin2, HIGH);
//                  // Calculating the distance
//                  distance2= duration2*0.034/2.0;
//                  distance2 = (int) distance2;    //convert to int
//                
//            
//            //}
//                
//           
//              // Prints the distance on the Serial Monitor
//             Serial.print("Distance2:");
//             Serial.println(distance2);
//             if ((distance2 > 0) && distance2 < 30){
//              incomingByte = distance2;
//             }
//             else{
//              incomingByte = 252;
//             }
//             //send_to_pic2 = false;   //debug
//            // delay(500);      
//
//        }
//
//          if (send_to_pic3 == true && !incomingByte) {
//              Serial.println("loop3");
//              //delay(500);
//
//             // for (int i = 0; i<5; i++){  //do five times
//                //Sensor 3
//                  // Clears the trigPin
//                  digitalWrite(trigPin3, LOW);
//                  delayMicroseconds(2);
//                  // Sets the trigPin on HIGH state for 10 micro seconds
//                  digitalWrite(trigPin3, HIGH);
//                  delayMicroseconds(10);
//                  digitalWrite(trigPin3, LOW);
//                  // Reads the echoPin, returns the sound wave travel time in microseconds
//                  duration3 = pulseIn(echoPin3, HIGH);
//                  //Serial.println(duration3);
//                  // Calculating the distance
//                  distance3= duration3*0.034/2.0;
//                  distance3 = (int) distance3;    //convert to int
//          //  }
//
//             Serial.print("Distance3:");
//             Serial.println(distance3);
//
//             if ((distance3 > 0) && distance3 < 30){
//              incomingByte = distance3;
//             }
//             else{
//              incomingByte = 254;
//             }
//             //send_to_pic3 = false;   //debug
//          }
//
//
//
//
//          
//          //initialize variables
////          int difference_tire_1 = 0;
////          int difference_tire_2 = 0;
////  
////          
////          difference_tire_1 = distance2 - base_radius;    //if tire is there then this distance should be almost the same as the distance1
////          difference_tire_2 - distance3 - base_radius;    //if tire is there then this distance should be almost the same as the distance1
////          Serial.println("diff 1 = "); 
////          Serial.println(difference_tire_1);
////          Serial.println("diff 2 = "); 
////          Serial.println(difference_tire_2);
////  
////          //check if tire there and set flags accordingly
////          if ( (distance2 < 30) && (abs(difference_tire_1-distance1) < 1) ){    //check if less than 30 cm and if 
////            incomingByte = incomingByte || (0xF);    //load 1111 into lower 4 bits of 8-bit incomingByte; this will tell the pic that there is a tire in the lower slot of the base
////          //incomingByte = 5;
////                   Serial.println("Incoming Byte Test:");
////                   Serial.println(incomingByte);
////
////          }
////            
////          if ( (distance3 < 30) && (abs(difference_tire_2-distance1) < 1) ){    //check if less than 30 cm and if 
////            incomingByte = incomingByte || (0xF<<4);    //load 1111 into upper 4 bits of 8-bit incomingByte; this will tell the pic that there is a tire in the lower slot of the base
////                    //incomingByte = 6;
////          }
//          //incomingByte = distance2;
//      }
//
//      
//  }
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
/////debug junkyard///
//
////for iterating through a loop
//
////int cnt = 0;
////while(cnt<5){
////      for (int i =0; i<5; i++){
////        Serial.println("Value of index:");
////        Serial.println(i);
////        Serial.println("mySensVals value:");
////        Serial.println(mySensVals[i]);
////        cnt+=1;
////        delay(500);
////      }
////}
////while(1){
////  Serial.println{"done");
////}
//



