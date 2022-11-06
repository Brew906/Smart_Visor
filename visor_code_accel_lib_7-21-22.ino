
//#include <Wire.h> //already included in the liquid crystal library
#include <DFRobot_QMC5883.h>
#include <AccelStepper.h>
#include <LiquidCrystal_I2C.h>


LiquidCrystal_I2C lcd(0x27, 27,2); 
DFRobot_QMC5883 compass;

#define motorInterfaceType 1

// defines pins numbers
const int stepPin = 12; 
const int dirPin = 14; 
const int photoPin = 34;
const int potPin = 32;

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
 
void setup() {
   
  // Sets the pin modes
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  pinMode(photoPin, INPUT);
  pinMode(potPin, INPUT);
  
  Wire.begin(21,22);
  
  Serial.begin(115200);

////// lcd display set up//////////
  lcd.init();
  lcd. backlight ();
  lcd.clear();
  lcd. print ( "  Smart Visor" );
  lcd. setCursor (2, 1);
  lcd.print("Michael Brewer");
  delay(2000);
  lcd.clear();
  lcd.setCursor(1,0);
  lcd.print("clock:");
  lcd.setCursor(8,0);
  lcd.print("5:00p.m.");
  delay(2000);
  lcd.clear();
  lcd.setCursor(1,0);
  lcd.print("time:");
  lcd.setCursor(9,0);
  lcd.print("Sun:");
  lcd.setCursor(1,1);
  lcd.print("Heading:");
/////////////////////////////////////
  motor_Setup(); 
/////////////////////////////////// 
   while (!compass.begin())
  {
    Serial.println("Could not find a valid QMC5883 sensor, check wiring!");
    delay(500);
  }///end of while statment
   if(compass.isHMC()){
        Serial.println("Initialize HMC5883");
        compass.setRange(HMC5883L_RANGE_1_3GA);
        compass.setMeasurementMode(HMC5883L_CONTINOUS);
        compass.setDataRate(HMC5883L_DATARATE_15HZ);
        compass.setSamples(HMC5883L_SAMPLES_8);
    }///end of if statment
   else if(compass.isQMC()){
        Serial.println("Initialize QMC5883");
        compass.setRange(QMC5883_RANGE_2GA);
        compass.setMeasurementMode(QMC5883_CONTINOUS); 
        compass.setDataRate(QMC5883_DATARATE_50HZ);
        compass.setSamples(QMC5883_SAMPLES_8);
   }///end of else statment
    delay(1000);
}////end of setup//////////////////

////////prototype functions/////////
void move_Motor_Clockwise()
{
    stepper.moveTo(100);
    stepper.runToPosition();
}
void move_Motor_CounterClockwise()
{
    stepper.moveTo(0);
    stepper.runToPosition();
}
// motor setup function
void motor_Setup()
{
    stepper.setCurrentPosition(0);
    stepper.setMaxSpeed(200);
    stepper.setAcceleration(230);
}
///// end of prototype functions////////////

void loop() {
  int value = analogRead(photoPin);
  int potRead = analogRead(potPin);
  potRead = map(potRead, 0, 4095, 1, 24);
  delay(250);
  Serial.println(value);
  Serial.println(potRead);
  int timeInfo = potRead;
//////////////////////////////////// 
///////prints info to display screen////////
  lcd.setCursor(6,0);
  if (timeInfo < 1000) lcd.print(" ");
  if (timeInfo < 100) lcd.print(" ");
  if (timeInfo < 10) lcd.print(" ");  
  lcd.setCursor(6,0);
  lcd.print(timeInfo);
  lcd.setCursor(12,0);
  if (value < 1000) lcd.print("    ");
  if (value < 100) lcd.print("   ");
  if (value < 10) lcd.print("   "); 
  lcd.setCursor(12,0); 
  lcd.print(value);
/////////////////////////////////////
  Vector norm = compass.readNormalize();
  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);
  float declinationAngle = (9.0 + (56.0 / 60.0)) / (180 / PI);  //(4.0 + (26.0 / 60.0)) / (180 / PI);
  heading += declinationAngle;
  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0){
    heading += 2 * PI;
  }///end of if statment
  if (heading > 2 * PI){
    heading -= 2 * PI;
  }///end of if statment
///// Convert to degrees
 float headingDegrees = heading * 180/M_PI; 
  Serial.print("Heading = ");
  Serial.print(heading);
  Serial.print(" Degress = ");
  Serial.print(headingDegrees);
  Serial.println();
///////////////////////////////////////////
////prints compass to display screen///////
  lcd.setCursor(9,1);
  lcd.print(headingDegrees);
//////////////////////////////////
/////conditional statments to control visor motor//////////////
 if (headingDegrees >= 245 && headingDegrees <= 290 || headingDegrees >= 68 && headingDegrees <= 110){
        if (timeInfo >= 16 && timeInfo <= 19 || timeInfo >= 5 && timeInfo <= 10){
             move_Motor_Clockwise(); 
       }///end of nested if statment
      }//end of first if statment
   else if (value >= 3900) { 
       move_Motor_Clockwise();
    } // end of else if statment    
   else {
    move_Motor_CounterClockwise();
    }/// end of else statment
delay(100);
}////end of void loop 
