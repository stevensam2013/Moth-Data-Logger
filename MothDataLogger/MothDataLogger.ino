/* ------------------ Include Libraries --------------------
Include all libraries that the program uses here
*/
#include <Servo.h>

/* --------------- Pre-Processor directives ----------------
The #define statements can be used instead of declaring variables
if the value will not change. This reduces the amount of program
memory used.
*/

//Define Pins
#define BUTTONX_PIN    2     //Set the pin that button X connects to
#define BUTTONY_PIN    3     //Set the pin that button Y connects to
#define S1_PIN         4     //Set the pin that the relay S1 connects to
#define S2_PIN         5     //Set the pin that the relay S2 connects to
#define SERVO1_PIN     10    //Set the pin that servo 1 connects to
#define SERVO2_PIN     11    //Set the pin that servo 2 connects to

//Define Servo positions
#define POSITION_A     180   //Set the value of servo position A
#define POSITION_B     50   //Set the value of servo position B

//Define delay times
#define SERVO_ON_TIME  10000 //Set the amount of time that servos
                             //stay on for, in milliseconds     
                             
#define RELAY_ON_TIME  40000 //Set the amount of time that relays
                             //stay on for, in milliseconds     

                             
                                     
#define RELAY_ON       HIGH  //Define pin state that turns on the relay
#define RELAY_OFF      LOW   //Define pin state that turns off the relay


//This program will operate as a state machine, with the following states
#define STATE_0        0     //Initial state, just turned on
#define STATE_1        1     //State 1, moving to position A
#define STATE_2        2     //State 2, in position A
#define STATE_3        3     //State 3, Moving to position B
#define STATE_4        4     //State 4, In Position B
                           

/* ------------------- Global Variables --------------------
Variables created here will be accessible to all functions
throughout this program.
*/

//Create Servo objects
Servo servo1;
Servo servo2;

//Create variable to store the state of the system
int systemState;



/* -------------------- Setup Function ---------------------
This function is run once at the start of the program and is
used to intialise values and setup resources, such as IO pins,
servos and serial ports
*/
void setup() {

  //Set the initial system state
  systemState = STATE_0;
  
  //Setup the input pin for buttons
  pinMode(BUTTONX_PIN, INPUT);
  pinMode(BUTTONY_PIN, INPUT);
  
  //Setup the output pin for the relays
  pinMode(S1_PIN, OUTPUT);
  pinMode(S2_PIN, OUTPUT);
  
  //Set relays to be off initially
  digitalWrite(S1_PIN, RELAY_OFF);
  digitalWrite(S2_PIN, RELAY_OFF);
  
  //Attach the servos to their pins
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);

}


/* ------------------ Main Program Loop --------------------
This is the main loop of the program. After the setup function
has run, code in this function is run. when it has completed,
it will start again.
*/
void loop() {

   
  //Use a switch/case to determine the system state
  switch (systemState)
  {
    case STATE_0:
      //Initial State - Just been switched on

      //if button X has been pressed
      if (digitalRead(BUTTONX_PIN))
      {
        //Change the system state to state 1
        systemState = STATE_1;
      }
      //if button Y has been pressed
      else if (digitalRead(BUTTONY_PIN))
      {
        //Change the system state to state 3
        systemState = STATE_3;
      }
      
      break;
    case STATE_1:
      //State 1 - Moving to position A
      
      //Move servos to position A
      moveServos(POSITION_A);

      //Trigger relay S1
      triggerRelay(S1_PIN);
      
      //Change the system state to state 2
      systemState = STATE_2;
      
      break;
      
    case STATE_2:
      //State 2 - Currently in position A

      //if button Y has been pressed
      if (digitalRead(BUTTONY_PIN))
      {
        //Change the system state to state 3
        systemState = STATE_3;
      }
      
      break;
      
    case STATE_3:
      //State 3 - Moving to position B

      //Trigger relay S2
      triggerRelay(S2_PIN);
      
      //Move servos to position B
      moveServos(POSITION_B);

      //Change the system state to state 4
      systemState = STATE_4;
      
      break;

    case STATE_4:
      //State 4 - Currently in position B

      //if button X has been pressed
      if (digitalRead(BUTTONX_PIN))
      {
        //Change the system state to state 1
        systemState = STATE_1;
      }
      
      break;
      
    default: 
      // This code should not be reached unless systemState is corrupted

      // Reset to initial state
      systemState = STATE_0;
      
      break;
  }
}

/* ------------------ Move Servos --------------------
This function moves both servos to the position passed
to it. The function waits for the movement to complete
*/
void moveServos(int servoPosition)
{    
  //Set the servo positions
  servo1.write(servoPosition);
  servo2.write(servoPosition);
  
  //Wait until the servos have finished
  delay(SERVO_ON_TIME);

}

/* ------------------ Trigger Relay --------------------
This function triggers the relay identified by the relayId
argument. The relay comes on for a fixed time, then goes
off.
*/
void triggerRelay(int relayId)
{
  //Activate relay
  digitalWrite(relayId, RELAY_ON);

  //Wait
  delay(RELAY_ON_TIME);

  //Deactivate relay
  digitalWrite(relayId, RELAY_OFF);

}

