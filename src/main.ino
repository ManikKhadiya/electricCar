//#include <IRremote.h>
#include <Servo.h>
#include <Wire.h> //for lcd
/*USE WHEN UPLOADING CODE
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);
https://www.electronicwings.com/arduino/lcd-16x2-interfacing-with-arduino-uno#:~:text=LCD%2016x2%20is%20a%2016,be%20used%20for%20control%20purposes.
https://create.arduino.cc/projecthub/akshayjoseph666/interface-i2c-16x2-lcd-with-arduino-uno-just-4-wires-273b24
dl from https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library
*/
//Copy of Elec.Car Manik Khadiya but 4pin USsensor BY MANIK KHADIYA 041220210129AM
// ideas: make turn momments into a subroutine.
#define DRIVE_MOTOR_L_EN 5 //enable
#define DRIVE_MOTOR_L_1 8
#define DRIVE_MOTOR_L_2 9

#define DRIVE_MOTOR_R_EN 6 //enable
#define DRIVE_MOTOR_R_1 10
#define DRIVE_MOTOR_R_2 11

#define DRIVE_DIRECTION_FORWARD 0
#define DRIVE_DIRECTION_BACK 1
#define DRIVE_SPEED_F 180
#define DRIVE_SPEED_R 100

Servo SteeringServo;
#define STEERING_SERVO_PIN 2
#define STEERING_ANGLE_RIGHT_MAX 117
#define STEERING_ANGLE_RIGHT_HALF 101
#define STEERING_ANGLE_LEFT_MAX 55
#define STEERING_ANGLE_LEFT_HALF 71
#define STEERING_ANGLE_MIDDLE 87

#define ULTRASONIC_SENSOR_TRIGGER 3
#define ULTRASONIC_SENSOR_ANALOG_IN A0
#define redLED 13

/*LCD INFO
Analog Pin 4 - SDA

Analog pin 5 - SCL

5V - Vcc

GND - GND*/

#define PIRpin 7

//vars
int t = 1000;
//-----------------------
void setup(){
    Serial.begin(9600);
  	Wire.begin();
  
    initialiseMotors();
    initialiseSteering();
    initUSsensor();
    initPIRsensor();
  	Serial.println("\nI2C Scanner");
  
  	/*lcd.backlight();
	lcd.clear();
	lcd.setCursor(4,0);
	lcd.print("Is Working?");
    ADD WHEN UPLOADING*/ 
}
void loop(){
    //testSteering();
    testDriveMotors();
    testUSsensor();
    testPIRsensor();
  	lcdDisplay();
}
void initialiseMotors(void){
  //DDRD=0b01100000; 
  //DDRB=0b00001111; 
  
  pinMode(DRIVE_MOTOR_L_EN, OUTPUT);
  pinMode(DRIVE_MOTOR_L_1 , OUTPUT);
  pinMode(DRIVE_MOTOR_L_2 , OUTPUT);
  
  pinMode(DRIVE_MOTOR_R_EN, OUTPUT);
  pinMode(DRIVE_MOTOR_R_1 , OUTPUT);
  pinMode(DRIVE_MOTOR_R_2 , OUTPUT);
  
  setDriveMotors(DRIVE_DIRECTION_FORWARD, DRIVE_SPEED_F);

}
void setDriveMotors(int direction, int speed){
  if(direction == DRIVE_DIRECTION_FORWARD){

      digitalWrite(DRIVE_MOTOR_L_1, HIGH);
      digitalWrite(DRIVE_MOTOR_L_2, LOW);
      digitalWrite(DRIVE_MOTOR_R_1, HIGH);
      digitalWrite(DRIVE_MOTOR_R_2, LOW);
      
    }else if (direction == DRIVE_DIRECTION_BACK){
      
      digitalWrite(DRIVE_MOTOR_L_1, LOW);
      digitalWrite(DRIVE_MOTOR_L_2, HIGH);
      digitalWrite(DRIVE_MOTOR_R_1, LOW);
      digitalWrite(DRIVE_MOTOR_R_2, HIGH);
  }
  analogWrite(DRIVE_MOTOR_L_EN, speed);
  analogWrite(DRIVE_MOTOR_R_EN, speed);
}
void testDriveMotors(void){
  setDriveMotors(DRIVE_DIRECTION_FORWARD, DRIVE_SPEED_F);
  setDriveMotors(DRIVE_DIRECTION_FORWARD, DRIVE_SPEED_R);
}
void initialiseSteering(void){
    SteeringServo.attach(STEERING_SERVO_PIN);
    Servo_h(STEERING_SERVO_PIN);
}
void setSteering(int angle){
  SteeringServo.write(angle);
}
/*void testSteering(){ 
    SteeringServo.write(STEERING_ANGLE_RIGHT_HALF);
    delay(t);
    SteeringServo.write(STEERING_ANGLE_MIDDLE);
    delay(t);
  
    SteeringServo.write(STEERING_ANGLE_RIGHT_MAX);
    delay(t);
    SteeringServo.write(STEERING_ANGLE_MIDDLE);
    delay(t);
  
    SteeringServo.write(STEERING_ANGLE_LEFT_HALF);
    delay(t);
    SteeringServo.write(STEERING_ANGLE_MIDDLE);
    delay(t);
  
    SteeringServo.write(STEERING_ANGLE_LEFT_MAX);
    delay(t);
    SteeringServo.write(STEERING_ANGLE_MIDDLE);
    delay(t);
  
    SteeringServo.write(STEERING_ANGLE_MIDDLE);
    delay(t);
}*/
void initUSsensor(){
  pinMode(ULTRASONIC_SENSOR_TRIGGER, OUTPUT);
    pinMode(ULTRASONIC_SENSOR_ANALOG_IN, INPUT);
    pinMode(redLED, OUTPUT);
  
    digitalWrite(ULTRASONIC_SENSOR_TRIGGER, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_SENSOR_TRIGGER, HIGH);
    delayMicroseconds(5);
    digitalWrite(ULTRASONIC_SENSOR_TRIGGER, LOW);
  
    digitalWrite(redLED, HIGH);
    delay(t/2);
    digitalWrite(redLED, LOW);
}
void testUSsensor(){
    int randint;
    long distanceThreshold = 50;
  long cm, duration;
  
    digitalWrite(ULTRASONIC_SENSOR_TRIGGER, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_SENSOR_TRIGGER, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_SENSOR_TRIGGER, LOW);
    
    //duration=analogRead(ULTRASONIC_SENSOR_ANALOG_IN);
    duration=pulseIn(ULTRASONIC_SENSOR_ANALOG_IN, HIGH);
    //debug
    //Serial.print(" pulseIn:");
    //Serial.print(duration);
    //Serial.println();
  
  cm=conversion(duration);
  
  randint = rand() % 2+1;
  
  //debug 
    Serial.print(" Distance: ");
    Serial.print(cm);
    Serial.print("cm ");
    Serial.println();
  
    Serial.print(" randint = ");
    Serial.print(randint);
    Serial.println();
  
    if (cm<distanceThreshold){
      digitalWrite(redLED, HIGH);
        Serial.print(" USsensor: THRESHOLD! ");
        if (randint==1){
          setSteering(STEERING_ANGLE_LEFT_MAX);
        delay(t);
          Serial.print("leftMAX");//debug
        }else if(randint==2){
          setSteering(STEERING_ANGLE_RIGHT_MAX);
        delay(t);
          Serial.print("rightMAX");//debug
        }
    }else{
      digitalWrite(redLED, LOW);
        delayMicroseconds(10);
    }
  //delay(500);
}
long conversion(long duration){
  duration=(duration*0.034)/2;
  return(duration); //sound is 340m/s = 34ms/cm and two way trip so we half
}

void initPIRsensor(){
  pinMode(PIRpin, INPUT);
}

void testPIRsensor(){
  while (digitalRead(PIRpin)== HIGH){
  Serial.println("PIR: TRIGGERED!");
  }
}

void lcdDisplay(){
	byte error, address;
	int Devices;
	Serial.println("Scanning...");
	Devices = 0;
	for(address = 1; address < 127; address++ ){
		Wire.beginTransmission(address);
		error = Wire.endTransmission();
		if (error == 0){
			Serial.print("I2C device found at address 0x");
          	if (address<16){
				Serial.print("0");
				Serial.print(address,HEX);
				Serial.println("  !");
          		Devices++;}
		}else if (error==4){
      		Serial.print("Unknown error at address 0x");}
  			if (address<16){
				Serial.print("0");
              	Serial.println(address,HEX);}
		}
	}
	/*if (Devices == 0){
		Serial.println("No I2C devices found\n");
    }else{
		Serial.println("done\n");
      	delay(5000);}
}*/