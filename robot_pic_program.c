/**
 * @file
 * @author Ruchir Tullu
 * @author Elizabeth Pratt
 * @author Hanya Wahdan
 *
 * 
 * @defgroup PIC_I2C_Arduino
 * @brief Demonstrates communication between the primary PIC and the Arduino
 *        Nano, via I2C. To see the results on the Arduino Nano side, open the 
 *        Arduino sample program Arduino_I2C_PIC. If the PIC receives a triple-A
 *        sequence from the keypad, it changes from being a transmitter to being
 *        a receiver that displays data on the character LCD. To change it back,
 *        reset the PIC
 * 
 * Preconditions:
 * @pre PIC-Arduino link switches are enabled (ON) for A4 and A5 of the Arduino
 *      Nano (RC4 and RC3 for the primary PIC, respectively)
 * @pre PIC and Arduino Nano agree on the Arduino Nano's slave address (i.e. the
 *      same slave address is used in software)
 * @pre A PC is connected if the Arduino Nano serial monitor is to be used. Note
 *      that the serial monitor baud rate must be 9600 for this program
 */

#include <xc.h>
#include <stdbool.h>
#include <pic18f4620.h>
#include "configBits.h"
#include "lcd.h"
#include "interface.h"
#include "keypad.h"
#include "I2C.h"
#include "dc_motor.h"
#include "stepper.h"
#include "pincer.h"
#include "canister.h"

const char keys[] = "123A456B789C*0#D";

const char happynewyear[7] = {      //set the time and upload once
    0x00, // 45 Seconds 
    0x28, // 59 Minutes
    0x17, // 24 hour mode, set to 23:00
    0x01, // Sunday
    0x06, // 31st
    0x04, // December
    0x19  // 2018
};

int drive = 1;
int delaycounter = 1; 
bool location_of_pincer = 0;    //0 = stepper canister, 1 = pulley canister
bool at_start = true;
int tire1_there = 0;       //1 is bottom
int tire2_there = 0;       //2 is top
int tires_to_dispense = 2;
float total_distance = 0;       //initialize
int canister_tires = 15;     //start off with 14 tires in canisters and 1 on pincer
int dist_to_next_pole = 0;      //distance to next pole, reset to zero every time
int pole_location[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};     //10 poles maximum, this is the distance from startline
int pole_tires_initial[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};        // how many tires on the poles initially
int pole_tires_deployed[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};       // how many tires on the poles finally
int pole_counter = 1;             //how many poles          
long int pulse1 = 0;
long int pulse2 = 0;
bool stop_driving_flag = false;
float distance_counter = 0;
int counter_drive_forward = 0;
int dist_between_poles = 0;
bool reset_pulley_canister = false;      //false
unsigned char time[7]; // Create a byte array to hold time read from RTC
unsigned char time1[7]; // Create a byte array to hold time read from RTC
unsigned char time2[7]; // Create a byte array to hold time read from RTC
int minutes;
int seconds;
int pulse_counter;

void rtc_set_time(void);
int convert_to_seconds(unsigned char time_array[7]);

void main(void) {
    
    // RB1, RB4, RB5, RB6, RB7 as inputs (for keypad)
    
    // RD2 is the character LCD RS
    // RD3 is the character LCD enable (E)
    // RD4-RD7 are character LCD data lines
  TRISA = 0x00;
  TRISB = 0b11110011;   //0x00   
  TRISCbits.TRISC6 = 0;
  TRISCbits.TRISC7 = 0;
  TRISD = 0x00;
  TRISE = 0x00;
    
    
    LATA = 0x00;
    LATB = 0x00;
    LATC = 0x00;
    LATD = 0x00;
    LATE = 0x00;

    // Set all A/D ports to digital (pg. 222)
    ADCON1 = 0b00001111;
    
    initLCD();  

    const char keys[] = "123A456B789C*0#D";


    // Set all A/D ports to digital (pg. 222)
    ADCON1 = 0b00001111;

    //    // Enable RB1 (keypad data available) interrupt
    INT1IE = 1;
    
    //shaft encoders
    //interrupt configuration
    INT0IE = 1;
    
    // Initialize LCD
    initLCD();

    // Enable interrupts
    ei();
    
    // Write the address of the slave device, that is, the Arduino Nano. Note
    // that the Arduino Nano must be configured to be running as a slave with
    // the same address given here. Note that other addresses can be used if
    // desired, as long as the change is reflected on both the PIC and Arduino
    // ends
    // Initialize I2C Master with 100 kHz clock
    I2C_Master_Init(100000); 
    I2C_Master_Start();
    I2C_Master_Write(0b00010000); // 7-bit Arduino slave address + write
    I2C_Master_Stop();
    
    // Set the time in the RTC. To see the RTC keep time, comment this line out
    // after programming the PIC directly before with this line included
    //rtc_set_time();
    
    
    unsigned char mem[3]; // Initialize array to check for triple-A sequence
    unsigned char counter = 0; // Increments each time a byte is sent
    unsigned char keypress; // Stores the data corresponding to the last key press
    int data1; // Holds the data to be sent/received        //was an unsigned char
    int data2; // Holds the data to be sent/received
    int data3; // Holds the data to be sent/received
    int data4; // Holds the data to be sent/received
    int distance; //holds shaft encoder data
    bool send = true;
    bool stay_in_menu = true; //change the value of this to break out of interface; set false if start pressed [A]]

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  interface
    while (stay_in_menu == true) {
        // RD2 is the character LCD RS
        // RD3 is the character LCD enable (E)
        // RD4-RD7 are character LCD data lines
        LATD = 0x00;
//        TRISD = 0x00;
        
        /////////////////////////////LCD Starts/////////////////////

        display_main_screen(); // shows menu options 

        unsigned char keypress = read_keypress(); //read value of keypress

        Nop(); // Apply breakpoint here to prevent compiler optimizations

        char temp = keys[keypress]; // get value of key (char)

        int not_back = 1;

        if (temp == keys[0]) { // Past operations = [1]
            while (not_back == 1) {
                display_past_operations();   //computes amount of tires dropped 
                unsigned char keypress = read_keypress(); //read value of keypress       
                char temp = keys[keypress]; // get value of key (char)

                if (temp == keys[0]) { //tires dropped
                    while (not_back == 1) { //condition to make it stay in this loop
                        display_tires_dropped(pole_tires_initial, pole_tires_deployed);
                        unsigned char keypress = read_keypress(); //read value of keypress       
                        char temp = keys[keypress]; // get value of key (char)
                        
                        if (temp == keys[0]) {      //how many tires on poles initially
                            display_tires_initially(pole_tires_initial);
                            
                            unsigned char keypress = read_keypress(); //read value of keypress       
                            char temp = keys[keypress]; // get value of key (char)
                            
                            if (temp == keys[2]) { //back
                                display_past_operations();
                                not_back = 0;
                            }
                        }
                        else if (temp == keys[1]) { //how many tires on poles finally
                            display_tires_finally(pole_tires_deployed);
                            
                            unsigned char keypress = read_keypress(); //read value of keypress       
                            char temp = keys[keypress]; // get value of key (char)
                            
                            if (temp == keys[2]) { //back
                                display_past_operations();
                                not_back = 0;
                            }
                        }
                        
                        else if (temp == keys[2]) {
                            display_past_operations();
                            not_back = 0;
                        }
                    }
                }

                else if (temp == keys[1]) { //tires distance
                    while (not_back == 1) {
                        display_tires_distance(pole_location);
                        unsigned char keypress = read_keypress(); //read value of keypress       
                        char temp = keys[keypress]; // get value of key (char)

                        if (temp == keys[2]) { //back
                            display_past_operations();
                            not_back = 0;
                        }
                    }
                }

                else if (temp == keys[3]) { //total # supplied tires and operation time
                    while (not_back == 1) {
                        operation_time(minutes, seconds, pole_counter);   //gets time, number of poles
                        unsigned char keypress = read_keypress(); //read value of keypress       
                        char temp = keys[keypress]; // get value of key (char)

                        if (temp == keys[2]) { //back
                            display_past_operations();
                            not_back = 0;
                        }
                    }
                }
                else if (temp == keys[2]) { //back = 3
                    display_main_screen();
                    not_back = 0;
                }
            }
        }
        
        

        if (temp == keys[1]) {              //keys[1] = 2 on keypad; this is for resetting the canisters (pulley canister only))
            while (not_back == 1) {
                reset_pulley_canister = true;
                reset_canister_menu();      //go into canister menu
                unsigned char keypress = read_keypress(); //read value of keypress       
                char temp = keys[keypress]; // get value of key (char)

                if (temp == keys[2]) { //back
                    display_past_operations();
                    not_back = 0;
                }
                else if (temp == keys[0]) { //1
                    lcd_clear();
                    lcd_set_ddram_addr(LCD_LINE1_ADDR);
                    printf("Press to lower"); //Testing to see if it gets stuck in while loop
                    lcd_set_ddram_addr(LCD_LINE2_ADDR);
                    printf("Stepper canister"); //Testing to see if it gets stuck in while loop

                    while (PORTBbits.RB1 == 0) {
                        LATCbits.LATC7 = 0; //turn on down
                        LATCbits.LATC6 = 0; //turn off up
                        continue;
                    }
                    // Wait until the key has been released
                    while (PORTBbits.RB1 == 1) {
                        //reset_canister_pulley(); //reset the pulley canister
                        LATCbits.LATC7 = 1; //turn on down
                        LATCbits.LATC6 = 0; //turn off up
                        __delay_ms(40);
                        reset_pulley_canister = false;
                        if(PORTBbits.RB1 == 0) {
                            LATCbits.LATC7 = 0; //turn on down
                            LATCbits.LATC6 = 0; //turn off up
                            break;
                        }
                        continue;
                    }
                }

                else if (temp == keys[1]) { //2
                    lcd_clear();
                    lcd_set_ddram_addr(LCD_LINE1_ADDR);
                    printf("Press to raise"); //Testing to see if it gets stuck in while loop
                    lcd_set_ddram_addr(LCD_LINE2_ADDR);
                    printf("Pulley canister"); //Testing to see if it gets stuck in while loop

                    while (PORTBbits.RB1 == 0) {
                        LATCbits.LATC7 = 0; //turn on down
                        LATCbits.LATC6 = 0; //turn off up
                        continue;
                    }

                    // Wait until the key has been released
                    while (PORTBbits.RB1 == 1) {
                        //reset_canister_pulley(); //reset the pulley canister
                        LATCbits.LATC7 = 0; //turn on down
                        LATCbits.LATC6 = 1; //turn off up
                        __delay_ms(40);
                        reset_pulley_canister = false;
                        if (PORTBbits.RB1 == 0) {
                            LATCbits.LATC7 = 0; //turn on down
                            LATCbits.LATC6 = 0; //turn off up
                            break;
                        }
                        continue;
                    }
                }

                else if (temp == keys[3]) { //A
                    lcd_clear();
                    lcd_set_ddram_addr(LCD_LINE1_ADDR);
                    printf("Press to raise"); 
                    lcd_set_ddram_addr(LCD_LINE2_ADDR);
                    printf("Stepper canister"); 

                    while (PORTBbits.RB1 == 0) {
                        LATEbits.LATE2 = 0; //turn on down
                        LATBbits.LATB2 = 0; //turn off up
                        continue;
                    }

                    // Wait until the key has been released
                    while (PORTBbits.RB1 == 1) {
                        //reset_canister_pulley(); //reset the pulley canister
                        LATEbits.LATE2 = 0; //turn on down
                        LATBbits.LATB2 = 1; //turn off up
                        __delay_ms(40);
                        reset_pulley_canister = false;
                        if (PORTBbits.RB1 == 0) {
                            LATEbits.LATE2 = 0; //turn on down
                            LATBbits.LATB2 = 0; //turn off up
                            break;
                        }
                        continue;
                    }
                }
                
            }
        }
       
        
        if (temp == keys[3]) { //start == A
            while (not_back == 1) {
                robot_running_initial(); //it works now
                delay_10ms(200); // no arithmetic overflow


                ////////////////////Don't need this for the initial log////////////////////////

                //                unsigned char keypress = read_keypress();       //read value of keypress       
                //                char temp = keys[keypress];        // get value of key (char)
                //
                //                if (temp == keys[2]){       //back
                //                        display_main_screen();              //**** change this later****//
                //                        not_back = 0;
                //                    }
                ////////////////////////////////////////////////////////////////////////////////
                not_back = 0;
                stay_in_menu = false;
                //buggy code
                //not_back = 0;    

                //                int start_loop = 3;
                //                
                //                while(start_loop != 0){
                //                    start_loop = start_loop -1;
                //                    __delay_ms(100);
                //                }
                //                if (start_loop == 0 ) { stay_in_menu = false;}
                //            stay_in_menu = false;       //break out of loop if start pressed



            }
            //stay_in_menu = false;       //break out of loop if start pressed
        }

        if (temp == keys[4]) { //time from RTC
            while (not_back == 1) {
                //display_time(); //show time while key not pressed
                while (PORTBbits.RB1 == 0) {
                  display_time(); //show time while key not pressed
                  continue;
                }

                
                if (PORTBbits.RB1 == 1) {
                    display_past_operations();
                    not_back = 0;
                }
            }
        }

    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// End interface

    
    //Now it's broken out of the loop, should start driving soon
    
    ///////////////////////// TIME SETTING AND GETTING ///////////////////////////////
    // Reset RTC memory pointer
    I2C_Master_Start(); // Start condition
    I2C_Master_Write(0b11010000); // 7 bit RTC address + Write
    I2C_Master_Write(0x00); // Set memory pointer to seconds
    I2C_Master_Stop(); // Stop condition
    // Read current time
    I2C_Master_Start(); // Start condition
    I2C_Master_Write(0b11010001); // 7 bit RTC address + Read
    for (unsigned char i = 0; i < 6; i++) {
        time1[i] = I2C_Master_Read(ACK); // Read with ACK to continue reading
    }
    time1[6] = I2C_Master_Read(NACK); // Final Read with NACK
    I2C_Master_Stop(); // Stop condition

    // Print received data on LCD
//    lcd_home();
//    printf("%02x/%02x/%02x", time1[6], time1[5], time1[4]); // Print date in YY/MM/DD
//    lcd_set_ddram_addr(LCD_LINE2_ADDR);
//    printf("%02x:%02x:%02x", time1[2], time1[1], time1[0]); // HH:MM:SS
//    __delay_ms(1000);
    ///////////////////////////////////////////////////////////////////////////////////

    
    
///////////////// ASK FOR SENSOR 1///////////////////////////////////
////now send flag "A" to arduino to initiate other sensor readings
//I2C_Master_Start(); // Start condition
//I2C_Master_Write(0b00010000); // 7-bit Arduino slave address + write
//I2C_Master_Write(65); // Write key press data    66 = B in ASCII
//I2C_Master_Stop();
////
////I2C_Master_Start();
////I2C_Master_Write(0b00010001); // 7-bit Arduino slave address + Read
//////data = 'data';
////data1 = I2C_Master_Read(NACK); // Read one char only         WHAT IS THIS???????? not acknowledged
////I2C_Master_Stop();
//////////////////////////////////////////////////////////////////////
//__delay_ms(10);
////    
////    //sketch debug
///////////////// ASK FOR SENSOR 2///////////////////////////////////
////now send flag "B" to arduino to initiate other sensor readings
//I2C_Master_Start(); // Start condition
//I2C_Master_Write(0b00010000); // 7-bit Arduino slave address + write
//I2C_Master_Write(66); // Write key press data    66 = B in ASCII
//I2C_Master_Stop();
////
////I2C_Master_Start();
////I2C_Master_Write(0b00010001); // 7-bit Arduino slave address + Read
//////data = 'data';
////data2 = I2C_Master_Read(NACK); // Read one char only         WHAT IS THIS???????? not acknowledged
////I2C_Master_Stop();
//////////////////////////////////////////////////////////////////////
//__delay_ms(10);
////
////
///////////////// ASK FOR SENSOR 3///////////////////////////////////
//I2C_Master_Start(); // Start condition
//I2C_Master_Write(0b00010000); // 7-bit Arduino slave address + write
//I2C_Master_Write(67); // Write key press data    66 = B in ASCII
//I2C_Master_Stop();
////
////I2C_Master_Start();
////I2C_Master_Write(0b00010001); // 7-bit Arduino slave address + Read
//////data = 'data';
////data3 = I2C_Master_Read(NACK); // Read one char only         WHAT IS THIS???????? not acknowledged
////I2C_Master_Stop();
/////////////////////////////////////////////////////////////////////       //end sketch debug start calling
//__delay_ms(10);
//
//    
    
    /////////////////// call the break beam sensor////////////////////
    I2C_Master_Start(); // Start condition
    I2C_Master_Write(0b00010000); // 7-bit Arduino slave address + write
    I2C_Master_Write(68); // Write key press data  65 = "A" in ASCII
    I2C_Master_Stop();
   /////////////////////////////////////////////////////////////////// 
    
    while(drive == 1) {

        drive_motor1_forward();
        drive_motor2_forward();

                ///////////////////////get the break beam data//////////////////
                
                I2C_Master_Start();
                I2C_Master_Write(0b00010001); // 7-bit Arduino slave address + Read
                //data = 'data';
                data4 = I2C_Master_Read(NACK); // Read one char only         //break beam read
                I2C_Master_Stop();
              ////////////////////////////////////////////////////////////////
        lcd_set_ddram_addr(LCD_LINE1_ADDR);
        printf("dist_counter: %d", distance_counter);
        lcd_set_ddram_addr(LCD_LINE2_ADDR);
        printf("pulse_counter: %d", pulse1);
        //__delay_ms(10);     //print debug, but remove from final version because it delays the stopping
       
            if ((data4 == 200) || (distance_counter > 400)) {      //if the break beam is broken or the total distance exceeds 400 cm
                drive = 0;
            }
                
            if (data4 == 100){
                continue;
            }
    
            counter_drive_forward += 1;
    }
    
    

    if (drive == 0) {
        lcd_clear();
        //data1 = 0;

        for (delaycounter = 3; delaycounter > 0; delaycounter--) {      //turn off stuff
            turn_off_motor1();
            turn_off_motor2();
            LATAbits.LATA6 = 0;
            LATAbits.LATA7 = 0;
            LATAbits.LATA4 = 0;
            LATAbits.LATA5 = 0;
            lcd_set_ddram_addr(LCD_LINE1_ADDR);
            printf("pole detected");
            lcd_set_ddram_addr(LCD_LINE2_ADDR);
            printf("%d",data4);
            __delay_ms(500);
        }
//        lcd_set_ddram_addr(LCD_LINE3_ADDR);
//        printf("distance:");
//        lcd_set_ddram_addr(LCD_LINE4_ADDR);
//        printf("%d", total_distance);
//        __delay_ms(400);

            ///////////// ASK FOR SENSOR 1///////////////////////////////////
            //now send flag "A" to arduino to initiate other sensor readings
            I2C_Master_Start(); // Start condition
            I2C_Master_Write(0b00010000); // 7-bit Arduino slave address + write
            I2C_Master_Write(65); // Write key press data    66 = B in ASCII
            I2C_Master_Stop();
                //call until not recieved 'a' signal from arduino telling it it's done sending
            I2C_Master_Start();
            I2C_Master_Write(0b00010001); // 7-bit Arduino slave address + Read
            //data = 'data';
            data1 = I2C_Master_Read(NACK); // Read one char only         WHAT IS THIS???????? not acknowledged
            I2C_Master_Stop();
            //data1 = (float) data1 / 10;     //debug for multiply by 10 here and divide by 10 on pic to retain decimal point
//            lcd_set_ddram_addr(LCD_LINE4_ADDR);     //debug
//            printf("dataone %f", data1);
//            __delay_ms(400);
//            lcd_clear();
            
            //////////////////////////////////////////////////////////////////
        
       

        //ultrasonic 1 printing logic and stopping logic
                    if((data1>0) && (data1<30)){
                        lcd_clear();
                        lcd_home(); 
                        lcd_set_ddram_addr(LCD_LINE1_ADDR);
        //                printf("data<30!!");
                        //__delay_ms(300);            //it works! 
                        //data  = "123A456B789C*0#D";
                        lcd_set_ddram_addr(LCD_LINE1_ADDR);
                        printf("sensor 1");
                        //__delay_ms(500);  
                    }
        
                        lcd_set_ddram_addr(LCD_LINE2_ADDR);
                        printf("%f", (float)data1);     //prints 97 when data = 'data' and flag is %d; a if flag is %c
                        
                        
                        if (((data1>0) && (data1<18)) || ((data1>0) && (data1> 150))) {      //changed from 21
                        lcd_set_ddram_addr(LCD_LINE3_ADDR);
                        printf("base");
                        }
        
                        __delay_ms(600);
        
        
               // __delay_ms(500);  
                /////////////////////////
        ///////////// ASK FOR SENSOR 2///////////////////////////////////
        //now send flag "B" to arduino to initiate other sensor readings
        I2C_Master_Start(); // Start condition
        I2C_Master_Write(0b00010000); // 7-bit Arduino slave address + write
        I2C_Master_Write(66); // Write key press data    66 = B in ASCII
        I2C_Master_Stop();
           //call until not recieved 'b' signal from arduino telling it it's done sending
        //while((data2 != 0) || (data2 != 1) || (data2 != 2) ){
            I2C_Master_Start();
            I2C_Master_Write(0b00010001); // 7-bit Arduino slave address + Read
            //data = 'data';
            data2 = I2C_Master_Read(NACK); // Read one char only         WHAT IS THIS???????? not acknowledged
            I2C_Master_Stop();
        //}
           //data2 = (float) data2 / 10; //debug for multiply by 10 here and divide by 10 on pic to retain decimal point
            
            //sensor logic on arduino testing
            if (data2 == 0) {
                lcd_clear();
                lcd_set_ddram_addr(LCD_LINE3_ADDR);
                printf("No tires");
                lcd_set_ddram_addr(LCD_LINE4_ADDR);
                printf("Result: %d",data2);
                __delay_ms(700);
            }
            if (data2 == 1){
                lcd_clear();
                lcd_set_ddram_addr(LCD_LINE3_ADDR);
                printf("One tire there");
                 lcd_set_ddram_addr(LCD_LINE4_ADDR);
                printf("Result: %d",data2);
                __delay_ms(700);
            }
            if (data2 == 2){
                lcd_clear();
                lcd_set_ddram_addr(LCD_LINE3_ADDR);
                printf("Two tires there");
                 lcd_set_ddram_addr(LCD_LINE4_ADDR);
                printf("Result: %d",data2);
                __delay_ms(700);
            }
            //////////////////////////////////////////////////////////////////
            
        lcd_clear();
        lcd_set_ddram_addr(LCD_LINE1_ADDR);
        printf("result");
        lcd_set_ddram_addr(LCD_LINE2_ADDR);
        printf("%d", (int) data2); //prints 97 when data = 'data' and flag is %d; a if flag is %c
        __delay_ms(700);
//        if ((data2 > 0) && (data2 < 30)) {
//            lcd_set_ddram_addr(LCD_LINE3_ADDR);
//            printf("tire1");
//            
//        }
 
    }
        
        
        

//        ///////////// ASK FOR SENSOR 3///////////////////////////////////
//        I2C_Master_Start(); // Start condition
//        I2C_Master_Write(0b00010000); // 7-bit Arduino slave address + write
//        I2C_Master_Write(67); // Write key press data    66 = B in ASCII
//        I2C_Master_Stop();
//        
//        I2C_Master_Start();
//        I2C_Master_Write(0b00010001); // 7-bit Arduino slave address + Read
//        //data = 'data';
//        data3 = I2C_Master_Read(NACK); // Read one char only         //break beam read
//        I2C_Master_Stop();
//        
//        lcd_clear();
//        lcd_set_ddram_addr(LCD_LINE1_ADDR); //debug
//        printf("breakbeam2: %d", data3);
//        __delay_ms(700);
//        
         //while (data3 != 116) { //call until not recieved 'c' signal from arduino telling it it's done sending
//            I2C_Master_Start();
//            I2C_Master_Write(0b00010001); // 7-bit Arduino slave address + Read
//            //data = 'data';
//            data3 = I2C_Master_Read(NACK); // Read one char only         WHAT IS THIS???????? not acknowledged
//            I2C_Master_Stop();
//            //data3 = (float) data3 / 10; //debug for multiply by 10 here and divide by 10 on pic to retain decimal point
//            lcd_set_ddram_addr(LCD_LINE4_ADDR); //debug
//            printf("datathree %f", data3);
//            __delay_ms(400);
//            lcd_clear();
//         //}
    
//        /////////////////////////////////////////////////////////////////
//        //lcd_clear();
//        //lcd_home();
//        //lcd_set_ddram_addr(LCD_LINE1_ADDR);
//        //                printf("data<30!!");
//        //__delay_ms(300);            //it works! 
//        //data  = "123A456B789C*0#D";
//        lcd_set_ddram_addr(LCD_LINE1_ADDR);
//        printf("sensor 3");
//        //__delay_ms(500);  
//        //
//
//        lcd_set_ddram_addr(LCD_LINE2_ADDR);
//        printf("%f", (float) data3); //prints 97 when data = 'data' and flag is %d; a if flag is %c
//
//        if ((data3 > 0) && (data3 < 30)) {
//            lcd_set_ddram_addr(LCD_LINE3_ADDR);
//            printf("tire2");
//        }
//
//        __delay_ms(600);
//                //////////////////////////
                
         
        
        
        //note: determine distance from last pole
       // tires_to_dispense = tires_to_dispense - tire1_there - tire2_there;      

    tires_to_dispense = 2 - data2;
    
        bool first_tire_ever = true;
        while(tires_to_dispense != 0){
            if (first_tire_ever == true) {
                drive_stepper_backward_start (data1);       //position over pole
                pincer_up();
                return_to_canister_stepper(data1);      //accounts for base radius and canister distance
                location_of_pincer = 0; // note pincers location at stepper
                tires_to_dispense = tires_to_dispense - 1;
                canister_tires -= 1;
                pole_tires_deployed[0] += 1;        //deployed 1 tire  to first pole 
            }
            if (first_tire_ever == false) {
                pincer_down();                      // get tire on pincer
                drive_stepper_forward(data1);       // position over pole
                pincer_up();                        // release tire onto pole
                drive_canister_stepper();           // drives canister up
                return_to_canister_pulley(data1);   // pincer reset to pulley end
                location_of_pincer = 1;             // note pincers location at pulley
                tires_to_dispense = tires_to_dispense - 1;
                canister_tires -= 1;
                pole_tires_deployed[0] += 1; //deployed 1 tire  to first pole 
                
            }   
            first_tire_ever = false;
        }
        pole_tires_initial[0] = 2 - pole_tires_deployed[0];     //number of tires initially on first pole
        tires_to_dispense = 2;      //reset
        
        
        
//debug
    lcd_set_ddram_addr(LCD_LINE3_ADDR);
    printf("done 1st loop:");
    __delay_ms(300);
        
        
        
        
        
        
        
        
        
        
        
        
        
//////////////////////////// MAIN LOOP FOR OTHER THAN START CONDITION//////////////////
    while ((distance_counter<400) && (canister_tires != 0) && (stop_driving_flag == false)){        //total_distance before
        printf("Second loop");
        __delay_ms(300);
        tires_to_dispense = 2; //reset
        drive = 1;
        
//        ///////////// ASK FOR SENSOR 1///////////////////////////////////       //sketch debug
//        //now send flag "A" to arduino to initiate other sensor readings
//        I2C_Master_Start(); // Start condition
//        I2C_Master_Write(0b00010000); // 7-bit Arduino slave address + write
//        I2C_Master_Write(65); // Write key press data    66 = B in ASCII
//        I2C_Master_Stop();
//
//        I2C_Master_Start();
//        I2C_Master_Write(0b00010001); // 7-bit Arduino slave address + Read
//        //data = 'data';
//        data1 = I2C_Master_Read(NACK); // Read one char only         WHAT IS THIS???????? not acknowledged
//        I2C_Master_Stop();
//        //////////////////////////////////////////////////////////////////
//
//        //sketch debug
//        ///////////// ASK FOR SENSOR 2///////////////////////////////////
//        //now send flag "B" to arduino to initiate other sensor readings
//        I2C_Master_Start(); // Start condition
//        I2C_Master_Write(0b00010000); // 7-bit Arduino slave address + write
//        I2C_Master_Write(66); // Write key press data    66 = B in ASCII
//        I2C_Master_Stop();
//
//        I2C_Master_Start();
//        I2C_Master_Write(0b00010001); // 7-bit Arduino slave address + Read
//        //data = 'data';
//        data2 = I2C_Master_Read(NACK); // Read one char only         WHAT IS THIS???????? not acknowledged
//        I2C_Master_Stop();
//        //////////////////////////////////////////////////////////////////
//
//
//        ///////////// ASK FOR SENSOR 3///////////////////////////////////
//        I2C_Master_Start(); // Start condition
//        I2C_Master_Write(0b00010000); // 7-bit Arduino slave address + write
//        I2C_Master_Write(67); // Write key press data    66 = B in ASCII
//        I2C_Master_Stop();
//
//        I2C_Master_Start();
//        I2C_Master_Write(0b00010001); // 7-bit Arduino slave address + Read
//        //data = 'data';
//        data3 = I2C_Master_Read(NACK); // Read one char only         WHAT IS THIS???????? not acknowledged
//        I2C_Master_Stop();
//        /////////////////////////////////////////////////////////////////       //end sketch debug start calling
        

        
        /////////////////// call the break beam sensor////////////////////
        I2C_Master_Start(); // Start condition
        I2C_Master_Write(0b00010000); // 7-bit Arduino slave address + write
        I2C_Master_Write(68); // Write key press data  65 = "A" in ASCII
        I2C_Master_Stop();
        /////////////////////////////////////////////////////////////////// 
        lcd_clear();    //debug for drive loop
        bool first_case = true;
        while (drive == 1) {

            drive_motor1_forward();
            drive_motor2_forward();
            if(first_case == true){  //so that it doesn't always delay
                __delay_ms(230);        //this is to have it drive a bit before addressing pole, to prevent getting broken on same pole base
                first_case = false;
            }
            ///////////////////////get the break beam data//////////////////
            //                
            I2C_Master_Start();
            I2C_Master_Write(0b00010001); // 7-bit Arduino slave address + Read
            //data = 'data';
            data4 = I2C_Master_Read(NACK); // Read one char only         //break beam read
            I2C_Master_Stop();
            //              ////////////////////////////////////////////////////////////////
            
            
           // lcd_set_ddram_addr(LCD_LINE3_ADDR);
           // printf("dist_counter: %d", distance_counter);

            if ((data4 == 200) ){ //if break beam broken or if distance exceeds 400 cm         //|| (distance_counter > 400)
//                lcd_clear();
//                lcd_set_ddram_addr(LCD_LINE3_ADDR);
//                printf("pole");
                drive = 0;
                
                if(distance_counter <400){      //only increment pole and do this stuff if the total distance is less than 400
                    pole_counter += 1; //there is a pole detected by break beam
                    total_distance = distance_counter; //increment distance
                    pole_location[pole_counter - 1] = distance_counter;
                    dist_between_poles = distance_counter - pole_location[pole_counter - 2];
                }
            }

            else if (data4 == 100) {    //was an if
                continue;
            }

        }
        
       


        if ((drive == 0) && (stop_driving_flag == false) ) {
            lcd_clear();
            //data1 = 0;

           // for (delaycounter = 3; delaycounter > 0; delaycounter--) {
                turn_off_motor1();
                turn_off_motor2();
                LATAbits.LATA6 = 0;
                LATAbits.LATA7 = 0;
                LATAbits.LATA4 = 0;
                LATAbits.LATA5 = 0;
                lcd_set_ddram_addr(LCD_LINE1_ADDR);
                printf("pole detected");
                lcd_set_ddram_addr(LCD_LINE2_ADDR);
                printf("%d", data4);
                __delay_ms(500);
         //   }
            lcd_clear();
            lcd_set_ddram_addr(LCD_LINE1_ADDR);
            printf("drive: %d", drive); 
            lcd_set_ddram_addr(LCD_LINE1_ADDR);
            printf("stop_driving_flag: %d", stop_driving_flag);     //prints 0 or 1
            __delay_ms(500);
                
                
            lcd_set_ddram_addr(LCD_LINE3_ADDR);
            printf("distance:");
            lcd_set_ddram_addr(LCD_LINE4_ADDR);
            printf("%d", total_distance);
            __delay_ms(400);

            ///////////// ASK FOR SENSOR 1///////////////////////////////////
            //now send flag "A" to arduino to initiate other sensor readings
            I2C_Master_Start(); // Start condition
            I2C_Master_Write(0b00010000); // 7-bit Arduino slave address + write
            I2C_Master_Write(65); // Write key press data    66 = B in ASCII
            I2C_Master_Stop();

            I2C_Master_Start();
            I2C_Master_Write(0b00010001); // 7-bit Arduino slave address + Read
            //data = 'data';
            data1 = I2C_Master_Read(NACK); // Read one char only         WHAT IS THIS???????? not acknowledged
            I2C_Master_Stop();
            data1 = (float) data1 / 10; //debug for multiply by 10 here and divide by 10 on pic to retain decimal point
            //////////////////////////////////////////////////////////////////

            //ultrasonic 1 printing logic and stopping logic
            if ((data1 > 0) && (data1 < 30)) {
                lcd_clear();
                lcd_home();
                lcd_set_ddram_addr(LCD_LINE1_ADDR);
                //                printf("data<30!!");
                //__delay_ms(300);            //it works! 
                //data  = "123A456B789C*0#D";
                lcd_set_ddram_addr(LCD_LINE1_ADDR);
                printf("sensor 1");
                //__delay_ms(500);  
            }

            lcd_set_ddram_addr(LCD_LINE2_ADDR);
            printf("%f", (float) data1); //prints 97 when data = 'data' and flag is %d; a if flag is %c


            if (((data1 > 0) && (data1 < 18)) || ((data1 > 0) && (data1 > 150))) { //changed from 21
                lcd_set_ddram_addr(LCD_LINE3_ADDR);
                printf("base");
            }

            __delay_ms(600);


            // __delay_ms(500);  
            /////////////////////////
            ///////////// ASK FOR SENSOR 2///////////////////////////////////
            //now send flag "B" to arduino to initiate other sensor readings
            I2C_Master_Start(); // Start condition
            I2C_Master_Write(0b00010000); // 7-bit Arduino slave address + write
            I2C_Master_Write(66); // Write key press data    66 = B in ASCII
            I2C_Master_Stop();

            I2C_Master_Start();
            I2C_Master_Write(0b00010001); // 7-bit Arduino slave address + Read
            //data = 'data';
            data2 = I2C_Master_Read(NACK); // Read one char only         WHAT IS THIS???????? not acknowledged
            I2C_Master_Stop();
            data2 = (float) data2 / 10; //debug for multiply by 10 here and divide by 10 on pic to retain decimal point
            //////////////////////////////////////////////////////////////////


            //lcd_clear();
            //lcd_home();
            //lcd_set_ddram_addr(LCD_LINE1_ADDR);
            //                printf("data<30!!");
            //__delay_ms(300);            //it works! 
            //data  = "123A456B789C*0#D";
            lcd_set_ddram_addr(LCD_LINE1_ADDR);
            printf("sensor 2");

            //

            lcd_set_ddram_addr(LCD_LINE2_ADDR);
            printf("%f", (float) data2); //prints 97 when data = 'data' and flag is %d; a if flag is %c

            if ((data2 > 0) && (data2 < 30)) {
                lcd_set_ddram_addr(LCD_LINE3_ADDR);
                printf("tire1");

            }
            __delay_ms(600);
            // __delay_ms(600);

        


        ///////////// ASK FOR SENSOR 3///////////////////////////////////
        I2C_Master_Start(); // Start condition
        I2C_Master_Write(0b00010000); // 7-bit Arduino slave address + write
        I2C_Master_Write(67); // Write key press data    66 = B in ASCII
        I2C_Master_Stop();

        I2C_Master_Start();
        I2C_Master_Write(0b00010001); // 7-bit Arduino slave address + Read
        //data = 'data';
        data3 = I2C_Master_Read(NACK); // Read one char only         WHAT IS THIS???????? not acknowledged
        I2C_Master_Stop();
        data3 = (float) data3 / 10; //debug for multiply by 10 here and divide by 10 on pic to retain decimal point
        /////////////////////////////////////////////////////////////////
        //lcd_clear();
        //lcd_home();
        //lcd_set_ddram_addr(LCD_LINE1_ADDR);
        //                printf("data<30!!");
        //__delay_ms(300);            //it works! 
        //data  = "123A456B789C*0#D";
        lcd_set_ddram_addr(LCD_LINE1_ADDR);
        printf("sensor 3");
        //__delay_ms(500);  
        //

        lcd_set_ddram_addr(LCD_LINE2_ADDR);
        printf("%f", (float) data3); //prints 97 when data = 'data' and flag is %d; a if flag is %c

//        if ((data3 > 0) && (data3 < 30)) {
//            lcd_set_ddram_addr(LCD_LINE3_ADDR);
//            printf("tire2");
//        }

        __delay_ms(600);
        //////////////////////////
        
        
        
        ////////////////check if tires are there///////////////
//        if ((data2 - data1) <= 2.5) { //3.2 , 1
//            tire1_there = 1;
//            lcd_set_ddram_addr(LCD_LINE1_ADDR);
//            printf("is tire 1");
//        }
//
//        if ((data3 - data1) <= 2.5) { //3.2, 1 
//            tire2_there = 1;
//            lcd_set_ddram_addr(LCD_LINE2_ADDR);
//            printf("is tire 2");
//        }
//        __delay_ms(300);

        ////////////////////////////////////////////////////////
        
//        tires_to_dispense = tires_to_dispense - tire1_there - tire2_there; 
        tires_to_dispense = tires_to_dispense - data2;
        
        //dist_to_next_pole = distance; //old one

    if (dist_between_poles < 30){         //need to drop one tire
            tires_to_dispense -= 1; //only need to drop one tire
        }
        lcd_clear();
        lcd_set_ddram_addr(LCD_LINE4_ADDR);
        printf("tires_to_disp: %d", tires_to_dispense);
        __delay_ms(700);
        
//   if (dist_to_next_pole > 30){         //need to drop two tires
//
//
//    }
        ////////////////////data 2 printing/////////////////////////////////
        //sensor logic on arduino testing
        if (data2 == 0) {
            lcd_clear();
            lcd_set_ddram_addr(LCD_LINE3_ADDR);
            printf("No tires");
            lcd_set_ddram_addr(LCD_LINE4_ADDR);
            printf("Result: %d", data2);
            __delay_ms(700);
        }
        if (data2 == 1) {
            lcd_clear();
            lcd_set_ddram_addr(LCD_LINE3_ADDR);
            printf("One tire there");
            lcd_set_ddram_addr(LCD_LINE4_ADDR);
            printf("Result: %d", data2);
            __delay_ms(700);
        }
        if (data2 == 2) {
            lcd_clear();
            lcd_set_ddram_addr(LCD_LINE3_ADDR);
            printf("Two tires there");
            lcd_set_ddram_addr(LCD_LINE4_ADDR);
            printf("Result: %d", data2);
            __delay_ms(700);
        }
        //////////////////////////////////////////////////////////////////

        lcd_clear();
        lcd_set_ddram_addr(LCD_LINE1_ADDR);
        printf("result");
        lcd_set_ddram_addr(LCD_LINE2_ADDR);
        printf("%d", (int) data2); //prints 97 when data = 'data' and flag is %d; a if flag is %c
        __delay_ms(700);
        
        //////////////////////////////////////////////////////////////////////////////////////////////////

    //note: determine distance from last pole
        if (first_tire_ever == true){       //if the first case has two tires already then it does this 
            //bool first_tire_ever = true;
            while (tires_to_dispense > 0) {     //!=0 before
                if (first_tire_ever == true) {
                    drive_stepper_backward_start(data1); //position over pole
                    pincer_up();
                    return_to_canister_stepper(data1); //accounts for base radius and canister distance
                    location_of_pincer = 0; // note pincers location at stepper
                    tires_to_dispense = tires_to_dispense - 1;
                    canister_tires -= 1;
                    pole_tires_deployed[0] += 1; //deployed 1 tire  to first pole 
                }
                if (first_tire_ever == false) {
                    pincer_down(); // get tire on pincer
                    drive_stepper_forward(data1); // position over pole
                    pincer_up(); // release tire onto pole
                    drive_canister_stepper(); // drives canister up
                    return_to_canister_pulley(data1); // pincer reset to pulley end
                    location_of_pincer = 1; // note pincers location at pulley
                    tires_to_dispense = tires_to_dispense - 1;
                    canister_tires -= 1;
                    pole_tires_deployed[0] += 1; //deployed 1 tire  to first pole 

                }
                first_tire_ever = false;
            }
        }

        while (tires_to_dispense > 0) {     //was !=0 before, infinite tires
            lcd_set_ddram_addr(LCD_LINE4_ADDR);
            printf("loop to dispense");
            __delay_ms(300);
            if (location_of_pincer == 0) {
                pincer_down(); // get tire on pincer
                drive_stepper_forward(data1); // position over pole
                pincer_up(); // release tire onto pole
                drive_canister_stepper(); // drives canister up
                return_to_canister_pulley(data1); // pincer reset to pulley end
                location_of_pincer = 1; // note pincers location at pulley
                tires_to_dispense = tires_to_dispense - 1;
                canister_tires -= 1;
                pole_tires_deployed[pole_counter-1] += 1;   //indexing starts at 0
            }

            else if (location_of_pincer == 1) {     //was if before
                pincer_down(); // get tire on pincer
                drive_stepper_backward(data1); // position over pole
                pincer_up(); // release tire onto pole
                drive_canister_pulley(); // drives canister up
                return_to_canister_stepper(data1); // pincer reset to stepper end
                location_of_pincer = 0; // note pincers location at pulley
                tires_to_dispense = tires_to_dispense - 1;
                canister_tires -= 1;
                pole_tires_deployed[pole_counter - 1] += 1; //indexing starts at 0
            }
        }
        if ((dist_between_poles < 30)){         //need to drop one tire
            pole_tires_initial[pole_counter - 1] = 1 - pole_tires_deployed[0]; //number of tires initially on the pole
        }
        if (dist_between_poles > 30){         //need to drop two tire
            pole_tires_initial[pole_counter - 1] = 2 - pole_tires_deployed[0]; //number of tires initially on the pole
        }
        
    }
        

    }
    

    lcd_clear();
    lcd_set_ddram_addr(LCD_LINE1_ADDR);
    printf("Pole location:");
    lcd_set_ddram_addr(LCD_LINE2_ADDR);
    printf("%d, %d, %d, %d, %d", pole_location[0], pole_location[1], pole_location[2], pole_location[3], pole_location[4]);
    lcd_set_ddram_addr(LCD_LINE3_ADDR);
    printf("%d, %d, %d, %d, %d", pole_location[5], pole_location[6], pole_location[7], pole_location[8], pole_location[9]);
    __delay_ms(800);


    lcd_clear();
    lcd_set_ddram_addr(LCD_LINE1_ADDR);
    printf("Pole initial:");
    lcd_set_ddram_addr(LCD_LINE2_ADDR);
    printf("%d, %d, %d, %d, %d", pole_tires_initial[0], pole_tires_initial[1], pole_tires_initial[2], pole_tires_initial[3], pole_tires_initial[4]);
    lcd_set_ddram_addr(LCD_LINE3_ADDR);
    printf("%d, %d, %d, %d, %d", pole_tires_initial[5], pole_tires_initial[6], pole_tires_initial[7], pole_tires_initial[8], pole_tires_initial[9]);
    __delay_ms(800);

    lcd_clear();
    lcd_set_ddram_addr(LCD_LINE1_ADDR);
    printf("Pole deployed:");
    lcd_set_ddram_addr(LCD_LINE2_ADDR);
    printf("%d, %d, %d, %d, %d", pole_tires_deployed[0], pole_tires_deployed[1], pole_tires_deployed[2], pole_tires_deployed[3], pole_tires_deployed[4]);
    lcd_set_ddram_addr(LCD_LINE3_ADDR);
    printf("%d, %d, %d, %d, %d", pole_tires_deployed[5], pole_tires_deployed[6], pole_tires_deployed[7], pole_tires_deployed[8], pole_tires_deployed[9]);
    __delay_ms(800);
    
    
    //turn off driving motor pins manually
    LATAbits.LATA4 = 0;
    LATEbits.LATE1 = 0;
    __delay_ms(200);    //delay to stop it from jerking when going backwards
    
    ///////////////////////////////////////drive backwards/////////////////////////////
    while (pulse2 < pulse_counter) { //counter_drive_forward, (int) distance_counter    //was pulse1, changed to pulse_counter because it needs to go back the same as pulse1 at the time it stops
        // LATAbits.LATA5 = 1;     //back
        //LATEbits.LATE0 = 1;
        drive_motor1_backward();
        drive_motor2_backward();
        lcd_clear();
        lcd_set_ddram_addr(LCD_LINE1_ADDR);
        printf("Pulse2: %d", pulse2);
        lcd_set_ddram_addr(LCD_LINE2_ADDR);
        printf("Pulse1: %d", pulse1);
        lcd_set_ddram_addr(LCD_LINE3_ADDR);
        printf("P_counter: %d", pulse_counter);
        __delay_ms(15);     //16 is a lot behind start line so 15 should work well
        pulse2 += 1;
    }
    /////////////////////////////////////////////////////////////////////////////////////
    //turn off backward driving motors
    LATAbits.LATA5 = 0;
    LATEbits.LATE0 = 0;
    lcd_clear();

    ///////////////////////// TIME SETTING AND GETTING ///////////////////////////////
    // Reset RTC memory pointer
    I2C_Master_Start(); // Start condition
    I2C_Master_Write(0b11010000); // 7 bit RTC address + Write
    I2C_Master_Write(0x00); // Set memory pointer to seconds
    I2C_Master_Stop(); // Stop condition
    // Read current time
    I2C_Master_Start(); // Start condition
    I2C_Master_Write(0b11010001); // 7 bit RTC address + Read
    for (unsigned char i = 0; i < 6; i++) {
        time2[i] = I2C_Master_Read(ACK); // Read with ACK to continue reading
    }
    time2[6] = I2C_Master_Read(NACK); // Final Read with NACK
    I2C_Master_Stop(); // Stop condition

    // Print received data on LCD
    lcd_home();
    printf("%02x/%02x/%02x", time2[6], time2[5], time2[4]); // Print date in YY/MM/DD
    lcd_set_ddram_addr(LCD_LINE2_ADDR);
    printf("%02x:%02x:%02x", time2[2], time2[1], time2[0]); // HH:MM:SS
    __delay_ms(1000);
    ///////////////////////////////////////////////////////////////////////////////////


    lcd_clear();
    int seconds2 = convert_to_seconds(time2);
    int seconds1 = convert_to_seconds(time1);
    int time_elapsed = seconds2 - seconds1;

    int minutes = time_elapsed / 60;
    int seconds = time_elapsed % 60;
    
    lcd_set_ddram_addr(LCD_LINE4_ADDR);
    printf("Time:%d:%d", minutes, seconds);
    __delay_ms(500);        //remove later for final version
    
    stay_in_menu = true; //change the value of this to break out of interface; set false if start pressed [A]]
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  Final interface
    while (stay_in_menu == true) {
        // RD2 is the character LCD RS
        // RD3 is the character LCD enable (E)
        // RD4-RD7 are character LCD data lines
        LATD = 0x00;
        //        TRISD = 0x00;

        /////////////////////////////LCD Starts/////////////////////

        display_main_screen_final(); // shows menu options for after operation

        unsigned char keypress = read_keypress(); //read value of keypress

        Nop(); // Apply breakpoint here to prevent compiler optimizations

        char temp = keys[keypress]; // get value of key (char)

        int not_back = 1;

        if (temp == keys[0]) { // Past operations = [1]
            while (not_back == 1) {
                display_past_operations(); //computes amount of tires dropped 
                unsigned char keypress = read_keypress(); //read value of keypress       
                char temp = keys[keypress]; // get value of key (char)

                if (temp == keys[0]) { //tires dropped
                    while (not_back == 1) { //condition to make it stay in this loop
                        display_tires_dropped(pole_tires_initial, pole_tires_deployed);
                        unsigned char keypress = read_keypress(); //read value of keypress       
                        char temp = keys[keypress]; // get value of key (char)

                        if (temp == keys[0]) { //how many tires on poles initially
                            display_tires_initially(pole_tires_initial);

                            unsigned char keypress = read_keypress(); //read value of keypress       
                            char temp = keys[keypress]; // get value of key (char)

                            if (temp == keys[2]) { //back
                                display_past_operations();
                                not_back = 0;
                            }
                        } else if (temp == keys[1]) { //how many tires on poles finally
                            display_tires_finally(pole_tires_deployed);

                            unsigned char keypress = read_keypress(); //read value of keypress       
                            char temp = keys[keypress]; // get value of key (char)

                            if (temp == keys[2]) { //back
                                display_past_operations();
                                not_back = 0;
                            }
                        }
                        else if (temp == keys[2]) {
                            display_past_operations();
                            not_back = 0;
                        }
                    }
                }
                else if (temp == keys[1]) { //tires distance
                    while (not_back == 1) {
                        display_tires_distance(pole_location);
                        unsigned char keypress = read_keypress(); //read value of keypress       
                        char temp = keys[keypress]; // get value of key (char)

                        if (temp == keys[2]) { //back
                            display_past_operations();
                            not_back = 0;
                        }
                    }
                }
                else if (temp == keys[3]) { //total # supplied tires and operation time
                    while (not_back == 1) {
                        operation_time(minutes, seconds, pole_counter); //gets time, number of poles
                        unsigned char keypress = read_keypress(); //read value of keypress       
                        char temp = keys[keypress]; // get value of key (char)

                        if (temp == keys[2]) { //back
                            display_past_operations();
                            not_back = 0;
                        }
                    }
                } else if (temp == keys[2]) { //back = 3
                    display_main_screen();
                    not_back = 0;
                }
            }
        }



        if (temp == keys[1]) { //keys[1] = 2 on keypad; this is for resetting the canisters (pulley canister only))
            while (not_back == 1) {
                reset_pulley_canister = true;
                reset_canister_menu(); //go into canister menu
                unsigned char keypress = read_keypress(); //read value of keypress       
                char temp = keys[keypress]; // get value of key (char)

                if (temp == keys[2]) { //back
                    display_past_operations();
                    not_back = 0;
                } else if (temp == keys[0]) { //1
                    lcd_clear();
                    lcd_set_ddram_addr(LCD_LINE1_ADDR);
                    printf("Press to lower"); //Testing to see if it gets stuck in while loop
                    lcd_set_ddram_addr(LCD_LINE2_ADDR);
                    printf("Stepper canister"); //Testing to see if it gets stuck in while loop

                    while (PORTBbits.RB1 == 0) {
                        LATCbits.LATC7 = 0; //turn on down
                        LATCbits.LATC6 = 0; //turn off up
                        continue;
                    }
                    // Wait until the key has been released
                    while (PORTBbits.RB1 == 1) {
                        //reset_canister_pulley(); //reset the pulley canister
                        LATCbits.LATC7 = 1; //turn on down
                        LATCbits.LATC6 = 0; //turn off up
                        __delay_ms(40);
                        reset_pulley_canister = false;
                        if (PORTBbits.RB1 == 0) {
                            LATCbits.LATC7 = 0; //turn on down
                            LATCbits.LATC6 = 0; //turn off up
                            break;
                        }
                        continue;
                    }
                }
                else if (temp == keys[1]) { //2
                    lcd_clear();
                    lcd_set_ddram_addr(LCD_LINE1_ADDR);
                    printf("Press to raise"); //Testing to see if it gets stuck in while loop
                    lcd_set_ddram_addr(LCD_LINE2_ADDR);
                    printf("Pulley canister"); //Testing to see if it gets stuck in while loop

                    while (PORTBbits.RB1 == 0) {
                        LATCbits.LATC7 = 0; //turn on down
                        LATCbits.LATC6 = 0; //turn off up
                        continue;
                    }

                    // Wait until the key has been released
                    while (PORTBbits.RB1 == 1) {
                        //reset_canister_pulley(); //reset the pulley canister
                        LATCbits.LATC7 = 0; //turn on down
                        LATCbits.LATC6 = 1; //turn off up
                        __delay_ms(40);
                        reset_pulley_canister = false;
                        if (PORTBbits.RB1 == 0) {
                            LATCbits.LATC7 = 0; //turn on down
                            LATCbits.LATC6 = 0; //turn off up
                            break;
                        }
                        continue;
                    }
                }
                else if (temp == keys[3]) { //A
                    lcd_clear();
                    lcd_set_ddram_addr(LCD_LINE1_ADDR);
                    printf("Press to raise");
                    lcd_set_ddram_addr(LCD_LINE2_ADDR);
                    printf("Stepper canister");

                    while (PORTBbits.RB1 == 0) {
                        LATEbits.LATE2 = 0; //turn on down
                        LATBbits.LATB2 = 0; //turn off up
                        continue;
                    }

                    // Wait until the key has been released
                    while (PORTBbits.RB1 == 1) {
                        //reset_canister_pulley(); //reset the pulley canister
                        LATEbits.LATE2 = 0; //turn on down
                        LATBbits.LATB2 = 1; //turn off up
                        __delay_ms(40);
                        reset_pulley_canister = false;
                        if (PORTBbits.RB1 == 0) {
                            LATEbits.LATE2 = 0; //turn on down
                            LATBbits.LATB2 = 0; //turn off up
                            break;
                        }
                        continue;
                    }
                }

            }
        }


        if (temp == keys[3]) { //start == A
            while (not_back == 1) {
                robot_running_initial(); //it works now
                delay_10ms(200); // no arithmetic overflow


                ////////////////////Don't need this for the initial log////////////////////////

                //                unsigned char keypress = read_keypress();       //read value of keypress       
                //                char temp = keys[keypress];        // get value of key (char)
                //
                //                if (temp == keys[2]){       //back
                //                        display_main_screen();              //**** change this later****//
                //                        not_back = 0;
                //                    }
                ////////////////////////////////////////////////////////////////////////////////
                not_back = 0;
                stay_in_menu = false;
                //buggy code
                //not_back = 0;    

                //                int start_loop = 3;
                //                
                //                while(start_loop != 0){
                //                    start_loop = start_loop -1;
                //                    __delay_ms(100);
                //                }
                //                if (start_loop == 0 ) { stay_in_menu = false;}
                //            stay_in_menu = false;       //break out of loop if start pressed



            }
            //stay_in_menu = false;       //break out of loop if start pressed
        }

        if (temp == keys[4]) { //time from RTC
            while (not_back == 1) {
                //display_time(); //show time while key not pressed
                while (PORTBbits.RB1 == 0) {
                    display_time(); //show time while key not pressed
                    continue;
                }


                if (PORTBbits.RB1 == 1) {
                    display_past_operations();
                    not_back = 0;
                }
            }
        }
        
        
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// End of interface
    
    
    
}



void __interrupt() interruptHandler(void) {
    //shaft encoder
    if (INT0IF && INT0IE) {
        pulse1++;
        distance_counter += 1.045; //1.045cm/pulse
        INT0IF = 0;
        if ((int) distance_counter == 400) {        //pulse1
            drive = 0;
            pulse_counter = pulse1;     //get current value of pulse 1 to go backwards by same amount
            stop_driving_flag = true;   //suppresses deployment logic
        }
    }

    // Interrupt on change handler for RB1
    if (INT1IF) {
//        if (reset_pulley_canister == true){
//            while(PORTBbits.RB1 == 1){
//                LATCbits.LATC7 = 1;
//            }
//        }
        // Notice how we keep the interrupt processing very short by simply
        // setting a "flag" which the main program loop checks
        INT1IF = 0; // Clear interrupt flag bit to signify it's been handled    
    }
}

void rtc_set_time(void) {
    I2C_Master_Start(); // Start condition
    I2C_Master_Write(0b11010000); //7 bit RTC address + Write
    I2C_Master_Write(0x00); // Set memory pointer to seconds

    // Write array
    for (char i = 0; i < 7; i++) {
        I2C_Master_Write(happynewyear[i]); //writes to SSPBUF = serial port buffer for transmission 
    }

    I2C_Master_Stop(); //Stop condition
}

int convert_to_seconds(unsigned char time_array[7]) {
    int digit0 = time_array[0] & 0x0F; //seconds first digit
    int digit1 = (time_array[0] & 0xF0) >> 4; //seconds upper digit
    int digit2 = time_array[1] & 0x0F; //minutes first digit  
    int digit3 = (time_array[1] & 0xF0) >> 4; //minutes upper digit 
    int digit4 = time_array[2] & 0x0F; //hours first digit
    int digit5 = (time_array[2] & 0xF0) >> 4; //hours upper digit

    int seconds = digit0 + 10 * digit1;

    int minutes_to_seconds = 60 * (digit2 + 10 * digit3);

    int hours_to_seconds = 3600 * (digit4 + 10 * digit5);

    int total_seconds = seconds + minutes_to_seconds + hours_to_seconds;

    return total_seconds;

}
        




