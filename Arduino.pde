
//////////////////////////////////////////////////////////////////////////////////
//////                                                                	    //////                                                                                                                  
//////        EE3GP Group C                                                 //////
//////        The Luggage 2011/2012                                         //////
//////        Written by: Christopher Bibb & Amir Behzad                    //////
//////                                                                      //////
//////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>             	 	 // Import header that deals with port mapping and memory management
#include <PVision.h>          		 // PVision library to interface the Wii IR Camera via I2C. Opensource libraries.
//#include <SoftwareSerial.h> 	 // Import header for serial data communication with sabertooth module
#include <LiquidCrystal.h>    	 // Import header for 16x2 LCD compatable with Hitachi HD44780 driver

PVision ircam;                 		 // Create instance of the PVision object and name it ircam 
byte result;                  		 // A byte stores an 8-bit unsigned number, from 0 to 255. Used for the Wii Camera to 

#define ECHOPINleft 30        	 // Set the echo pin of the left ultrasonic sensor to pin 26
#define TRIGPINleft 31        	 // Set the trigger pin of the left ultrasonic sensor to pin 27
#define ECHOPINright 39     	 // Set the eco pin of the right ultrasonic sensor to pin 30
#define TRIGPINright 38       	 // Set the trigger pin of the right ultrasonic sensor to pin 31
#define ECHOPINback 47        	 // Set the echo pin of the back ultrasonic sensor to pin 39
#define TRIGPINback 46        	 // Set the trigger pin of the back ultrasonic sensor to pin 38
#define ECHOPINfront 26                   // Set the echo pin of the front ultrasonic sensor to pin 47
#define TRIGPINfront 27                     // Set the trigger pin of the front ultrasonic sensor to pin 46

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);  // Wiring pins for the LCD
boolean leftSensor, rightSensor, frontSensor, backSensor;    
// Define boolean variables for the 4 ultrasonic sensors to represent obstacle found (<20cm) and not found (>20cm)

#define SABER_TX_PIN 13                  // Labels for use with the Sabertooth 2x5 motor controller. Digital pin 13 is the serial transmit pin to the Sabertooth 2x5
#define SABER_RX_PIN 12                  // NOT USED (but still init'd). Digital pin 12 is the serial receive pin from the Sabertooth 2x5
#define SABER_BAUDRATE 9600      // Set to 9600 through Sabertooth dip switches


#define SABER_MOTOR1_FULL_FORWARD 127     // The sabertooth sets the speed by using the values to the left, such that 64 is stop, so if the value 65 was sent then
#define SABER_MOTOR1_FULL_REVERSE 1            // it would go very slow, and if the value 127 was sent it would go at full speed. Also, to slowly reverse the value 60 could be given.
#define SABER_MOTOR1_FULL_STOP 64                // These variables can be adjusted to decrease the motors maximum speed or forward and reverse .
	
#define SABER_MOTOR2_FULL_FORWARD 255    // The speeds for the two motors can be modified independantly. This is usefull because one motor is faster than the other, and by finding
#define SABER_MOTOR2_FULL_REVERSE 128      // the speed % differance it can be changed here so that the motors then run at the same speed regardless of any speed discrepancies.
#define SABER_MOTOR2_FULL_STOP 192
#define SABER_ALL_STOP 0                	        // Motor level to send when issuing the full stop command

SoftwareSerial SaberSerial = SoftwareSerial( SABER_RX_PIN, SABER_TX_PIN );    
// Create instance of SoftwareSerial for sabertooth to communicate with motors by

void setup()                       // This function runs once at start up. This is mainly used for set up and initialisation of classes to create a object.
{
  Serial.begin(9600);        // Baud rate of 9600 works for all the devices, including Wii Camera, Ultrasonic, LCD & Sabertooth data pins
  delay(2000);                   // 2 Seconds delay to allow initialisation of Arduino so that garbage data is not recieved
  pinMode(ECHOPINleft,INPUT);      // The echo pin of the left ultrasonic sensor is set up as an input, as it reads data from the ultrasonic sensor echo response
  pinMode(TRIGPINleft,OUTPUT);     // The trigger pin of the left ultrasonic sensor is set up as 
  pinMode(ECHOPINright,INPUT);
  pinMode(TRIGPINright,OUTPUT); 
  pinMode(ECHOPINback,INPUT);
  pinMode(TRIGPINback,OUTPUT); 
  pinMode(ECHOPINfront,INPUT);
  pinMode(TRIGPINfront,OUTPUT); 
  
  lcd.begin(16, 2);             	 // set up the LCD's number of columns and rows: 
  lcd.print("Power: ON);   	// Print a message to the LCD.
  ircam.init();       	              // Initialise the IR Camera by calling the function within the instance ircam of PVision
  initSabertooth( );                     // Call function initSabertooth below to initialise the sabertooth
  pinMode(52, OUTPUT);          // Set up a LED that can flash to indicate a IR blob is detected by the Wii Camera. Used for debugging
}

void loop()
{
  ////////////////////////////////////////////////////
  //               Left Ultrasonic Sensor                             //
  ////////////////////////////////////////////////////
  digitalWrite(TRIGPINleft, LOW);  // Set the trigger pin to low for 2uS
  delayMicroseconds(2);
  digitalWrite(TRIGPINleft, HIGH);  // Send 10uS high to trigger ranging
  delayMicroseconds(10);
  digitalWrite(TRIGPINleft, LOW);  // Send pin low again

  int distanceLeft = pulseIn(ECHOPINleft, HIGH);        // Read in times pulse
  distanceLeft = distanceLeft/58;                     	             // Calculate distance from time of pulse
  
  ////////////////////////////////////////////////////
  //              Right Ultrasonic Sensor                           //
  ////////////////////////////////////////////////////
  digitalWrite(TRIGPINright, LOW);  // Set the trigger pin to low for 2uS
  delayMicroseconds(2);
  digitalWrite(TRIGPINright, HIGH);  // Send 10uS high to trigger ranging
  delayMicroseconds(10);
  digitalWrite(TRIGPINright, LOW);  // Send pin low again

  int distanceRight = pulseIn(ECHOPINright, HIGH);        // Read in times pulse
  distanceRight= distanceRight/58;                        // Calculate distance from time of pulse

  ////////////////////////////////////////////////////
  //               Back Ultrasonic Sensor                           //
  ////////////////////////////////////////////////////
  digitalWrite(TRIGPINback, LOW);  			// Set the trigger pin to low for 2uS
  delayMicroseconds(2);
  digitalWrite(TRIGPINback, HIGH);  			// Send 10uS high to trigger ranging
  delayMicroseconds(10);
  digitalWrite(TRIGPINback, LOW); 			 // Send pin low again

  int distanceBack = pulseIn(ECHOPINback, HIGH);        // Read in times pulse
  distanceBack= distanceBack/58;                     	  // Calculate distance from time of pulse

  ////////////////////////////////////////////////////
  //               Front Ultrasonic Sensor                      //
  ////////////////////////////////////////////////////
  digitalWrite(TRIGPINfront, LOW); 			 // Set the trigger pin to low for 2uSt 
  delayMicroseconds(2);
  digitalWrite(TRIGPINfront, HIGH);  		// Send 10uS high to trigger ranging
  delayMicroseconds(10);
  digitalWrite(TRIGPINfront, LOW);			  // Send pin low again
  
  int distanceFront = pulseIn(ECHOPINfront, HIGH);      // Read in times pulse
  distanceFront= distanceFront/58;                       	 // Calculate distance from time of pulse
 
  ////////////////////////////////////////////////////
  //               Print Ultrasonic Data                               //
  ////////////////////////////////////////////////////
  Serial.print("  Left Ultrasonic (cm): ");                	// Print data used for debugging
  Serial.print(distanceLeft);
  Serial.print("  Right Ultrasonic (cm); ");
  Serial.print(distanceRight);
  Serial.print("  Back Ultrasonic (cm); ");
  Serial.print(distanceBack);
  Serial.print("  Front Ultrasonic (cm); ");
  Serial.println(distanceFront);
  
  ////////////////////////////////////////////////////
  //      Sensor Booleans for object detection          //
  ////////////////////////////////////////////////////
  if (distanceLeft < 20 && distanceLeft >= 1)      
  // This uses a boolean to check if the distance from the ultrasonic sensor is below 20 and aboove 1 (as sometimes the ultrasonic
    leftSensor = true;                                   // can return data below 0, such as -573, which is an error with the ultrasonoic sensors.
  else                                           	       // If the sensor is below 20 it will set it to true, to indicate it is detected, and if not then to false to indicate there is no obstacle
    leftSensor = false;
    
  if (distanceRight < 20 && distanceLeft >= 1) 
    rightSensor = true;
  else 
    rightSensor = false;
  
  if (distanceBack < 20 && distanceLeft >= 1) 
    backSensor = true;
  else 
    backSensor = false;
    
  if (distanceFront < 20 && distanceLeft >= 1) 
    frontSensor = true;
  else 
    frontSensor = false;
  
  /////////////////////////////////////////////////////////////////////
  //     Object Avoidance: Motor controlled by sensor feedback  //
  /////////////////////////////////////////////////////////////////////
  
  if ((leftSensor == true) || (frontSensor == true) || (rightSensor == true))      // obstacle detected
  {
    if((leftSensor == false) && (frontSensor == true) && (rightSensor == false)){            // obstacle just in middle, break   and follow forward
      setEngineSpeedDir( -40 ); 				 //Move Left Fast
      Serial.println(" Probably owner ");
    }
    
    if((leftSensor == true) && (frontSensor == false) && (rightSensor == true)){        // obstacle on left and right but not in middle, break and follow forward
      setEngineSpeedDir( -40 ); 				 //Move Left Fast
      Serial.println(" Squeeze through! ");
    }
    
    if ((leftSensor == true) && (frontSensor == true) && (rightSensor == false)) {           // obstacle on left
      // Move right LOTS
            setEngineSpeedDir( -40 ); 				 //Move Left Fast
            Serial.println(" -----------> ");
    }
    
    if ((leftSensor == false) && (frontSensor == true) && (rightSensor == true)){            // obstacle on right
      // Move left LOTS
      setEngineSpeedDir( -40 ); 				 //Move Left Fast     
      Serial.println(" <----------- ");
    }
    
    if ((leftSensor == true) && (frontSensor == false) && (rightSensor == false)){            // obstacle on far left
      // Move right
      setEngineSpeedDir( -40 ); 				 //Move Left Fast
      Serial.println(" ----> ");
    }
    
    if ((leftSensor == false) && (frontSensor == false) && (rightSensor == true)){            // obstacle on far right
      // Move left
      setEngineSpeedDir( -40 ); 				 //Move Left Fast
      Serial.println(" <---- ");
    }
  
    if ((leftSensor == true) && (frontSensor == true) && (distanceBack <50 && distanceBack>5)){            // obstacle on far right
      // Stop. Person/Obstacle walking between user and robot
      setEngineSpeedDir( -40 ); 				 //Move Left Fast
      Serial.println(" Stop ");
    }
    
    if ((leftSensor == true) && (frontSensor == true) && (rightSensor == true) && (backSensor == true)){            // obstacle on far right
      // Surrounded in all directions
        setEngineSpeedDir( -40 ); 				 //Move Left Fast
    Serial.println(" Surrender ");
    }
   }

  //////////////////////////////////////////////////////////////////////////
  //               Check Ultrasonic Sensors for Obstacles                               //
  /////////////////////////////////////////////////////////////////////////
 /*
  if ((leftSensor == true) || (rightSensor == true))      	        // obstacle detected
  {
    if ((leftSensor == true) && (rightSensor == false)) {           // obstacle on left
      // Move right LOTS
      setEngineSpeedDir( 20 );  //Move Forward
      Serial.println(" -----------> ");
      delay(500);
    }
    
    if ((leftSensor == false) && (rightSensor == true)){            // obstacle on right
      // Move left LOTS
      setEngineSpeedDir( -20 );  //Move Forward
      Serial.println(" <----------- ");
      delay(500);
    }  
     
    if ((leftSensor == true) && (rightSensor == true)){            // obstacle on right
      // Move left LOTS
      setEngineSpeedDir( -20 );  //Move Forward
      Serial.println(" XXXXXXXXXXXX ");
      delay(500);
    }  
  }
*/
  ////////////////////////////////////////////////////
  //               Wii IR Sensor Data                               //
  ////////////////////////////////////////////////////
  
   if ((distanceLeft >20) && (distanceRight >20)){      // Check if an obstacle is detected, and if not enter the IF statement
  
      result = ircam.read();  // Call read function from PVision.cpp and read data into result.
      
      if (result & BLOB1)					// Read X coordinates of IR LED to determine direction
      {
        digitalWrite(52, HIGH);   				// set the LED on to show data recieved
        Serial.print("   BLOB1 detected. X:");
        Serial.print(ircam.Blob1.X);
        Serial.print(" Y:");
        Serial.print(ircam.Blob1.Y);
        Serial.print(" Size:");
        Serial.println(ircam.Blob1.Size);
        digitalWrite(52, LOW);   				 // set the LED off
       }
   
          if ((ircam.Blob1.X > 550) && (ircam.Blob1.X < 700)){
            setEngineSpeedDir( 20 );  				//Move Right Slow
            Serial.print("RIGHT SLOW");
            Serial.print("   X =");
            Serial.println(ircam.Blob1.X);
            delay(500);
          }
          
          if ((ircam.Blob1.X > 700) && (ircam.Blob1.X < 1022)){
            setEngineSpeedDir( 40 );  				//Move Right Slow
            Serial.print("RIGHT FAST");
            Serial.print("   X = ");
            Serial.println(ircam.Blob1.X);
             delay(500);    
          }
          
          if ((ircam.Blob1.X < 450) && (ircam.Blob1.X > 300)){
           // setEngineSpeedDir( -20 ); 				 //Move Left Slow
            Serial.print("LEFT SLOW");
            Serial.print("   X = ");
            Serial.println(ircam.Blob1.X);
            delay(500);  
          }
          
          if ((ircam.Blob1.X < 300) && (ircam.Blob1.X > 1)){
           // setEngineSpeedDir( -40 ); 				 //Move Left Fast
            Serial.print("LEFT FAST");
            Serial.print("   X = ");
            Serial.println(ircam.Blob1.X);
            delay(500);  
          }
          
            /////////////////////////////////////////////
           //		Go Forward		   //
          //////////////////////////////////////////////

            if ((ircam.Blob1.X < 550) && (ircam.Blob1.X > 450)){	// If the X coordinate is 100 within the center then go forward
              
              if ((ircam.Blob1.X > 0) && (ircam.Blob2.X > 0)) { 	 // Uses 2 IR LED's to derive a accurate distance from a 2D matrix
              // Get distance from user using 2 IR LED's
                int x1 = ircam.Blob1.X;
                int x2 = ircam.Blob2.X;
                int y1 = ircam.Blob1.Y;
                int y2 = ircam.Blob2.Y;
                double distance;  
                
                // Use the point coordinated output by the wii camera (X and Y in a 2D plane), as a means
                // to derive distance using Euclidean vector space Cartesian coordinates.
                // d = sqrt[(x2-x1)^2 + (y2-y1)^2]
                
                distance = sqrt(sq(x2-x1) + sq(y2-y1));		// Calculate Euclidean Distance
                distance = map(distance, 0, 1023, 0, 100); 		// maximum values of distance decreased to 100 to be proportional to motor speed
                            
                setEngineSpeed( distance );  			//Move Forward at distance speed
                
                Serial.print("FORWARD 2 IR");
                Serial.print("   Distance = ");                                                                                                                                                                                              
                Serial.println(distance);
              }
            
              else if((ircam.Blob1.X > 0) && (distanceFront > 50))  {		// only finds 1 IR, so uses the front ultrasonic as backup
                
                setEngineSpeed((distanceFront-100)*1.2);  
//Move Forward at a linearly increasing speed. e.g. if user 120cm away then (120-100)*1.2 = 24 speed. (150-100)*1.2 = 60
                Serial.print("FORWARD 1 IR & Ultrasonic");
                Serial.print("  Distance = ");                                                                                                                                                                                              
                Serial.println(distanceFront);
              }
            } 
      }

  lcd.setCursor(0, 1);        			// set the cursor to column 0, line 1 (note: line 1 is the second row, since counting begins with 0):
  lcd.print(millis()/1000);   			// print the number of seconds since reset:}

//////////////////////////////////////////////////////////////
//		SABERTOOTH		                    //  
//////////////////////////////////////////////////////////////

void initSabertooth( void )
{
 // Init software UART to communicate  with the Sabertooth 2x5
 pinMode( SABER_TX_PIN, OUTPUT );
 SaberSerial.begin( SABER_BAUDRATE );

 // 2 second time delay for the Sabertooth to init
 delay( 500 );
 // Send full stop command
 setEngineSpeed( SABER_ALL_STOP );
}

void setEngineSpeed( signed char cNewMotorSpeed )
{
 unsigned char cSpeedVal_Motor1 = 0;
 unsigned char cSpeedVal_Motor2 = 0;

 // Check for full stop command
 if( cNewMotorSpeed == 0 )
 {
   // Send full stop command for both motors
   SaberSerial.print( 0, BYTE );
   return;
 }

 // Calculate the speed value for motor 1
 if( cNewMotorSpeed >= 100 )
 {
   cSpeedVal_Motor1 = SABER_MOTOR1_FULL_FORWARD;
   cSpeedVal_Motor2 = SABER_MOTOR2_FULL_FORWARD;
 }
 else if( cNewMotorSpeed <= -100 )
 {
   cSpeedVal_Motor1 = SABER_MOTOR1_FULL_REVERSE;
   cSpeedVal_Motor2 = SABER_MOTOR2_FULL_REVERSE;
 }
 else
 {
   // Calc motor 1 speed (Final value ranges from 1 to 127)
   cSpeedVal_Motor1 = map( cNewMotorSpeed, -100, 100, SABER_MOTOR1_FULL_REVERSE, SABER_MOTOR1_FULL_FORWARD );

    // Calc motor 2 speed (Final value ranges from 128 to 255)
    cSpeedVal_Motor2 = map( cNewMotorSpeed, -100,  100,  SABER_MOTOR2_FULL_REVERSE, SABER_MOTOR2_FULL_FORWARD );
 }

 // Fire the values off to the Sabertooth motor controller
 SaberSerial.print( cSpeedVal_Motor1, BYTE );
 SaberSerial.print( cSpeedVal_Motor2, BYTE );
}

void setEngineSpeedDir( signed char cNewMotorSpeedDir )
{
 unsigned char cSpeedValDir_Motor1 = 0;
 unsigned char cSpeedValDir_Motor2 = 0;
 if( cNewMotorSpeedDir >= 100 )
 {
   cSpeedValDir_Motor1 = SABER_MOTOR1_FULL_FORWARD; // GO RIGHT
   cSpeedValDir_Motor2 = SABER_MOTOR2_FULL_STOP;
 }
 else if( cNewMotorSpeedDir <= -100 )
 {
   cSpeedValDir_Motor1 = SABER_MOTOR1_FULL_STOP; // GO LEFT
   cSpeedValDir_Motor2 = SABER_MOTOR2_FULL_FORWARD;
 }
 else
 {
   // Calc motor 1 speed (Final value ranges from 64 to 127)
   cSpeedValDir_Motor1 = map( cNewMotorSpeedDir, -100, 100, SABER_MOTOR1_FULL_STOP, SABER_MOTOR1_FULL_FORWARD );

   // Calc motor 2 speed (Final value ranges from 192 to 255)
   cSpeedValDir_Motor2 = map( cNewMotorSpeedDir, -100, 100, SABER_MOTOR2_FULL_FORWARD, SABER_MOTOR2_FULL_STOP);
 }

 // Fire the values off to the Sabertooth motor controller
 SaberSerial.print( cSpeedValDir_Motor1, BYTE );
 SaberSerial.print( cSpeedValDir_Motor2, BYTE );
}

//    setEngineSpeed function dictates movement of forwards and backwards. Both motors will turn in the same directions when placed in opposite orientations. Negative value moves forward
//    Positive value moves backward Zero can be used to stop both motors instantly setEngineSpeedDir function dictates the direction the luggage
//    Both motors will turn in opposite directions when placed opposite orientations.
//    Negative value turns the luggage anticlockwise/left
//    Positive value turns the luggage clockwise/right
//    Values can be in the range from -100 to 100
//    The values correspond to the speed of the motors
