//Arduno firmware for Kondo-Pro
//vesion 0.9
//email="ryakin.is@phystech.edu" Ilya Ryakin
//StarkitRobots 2020
//bundrate =115200/1250000
//additional library https://github.com/adafruit/Adafruit-MCP23017-Arduino-Library.git  


/////////////////ROS2 Arduino stuff 


#include <Adafruit_MCP23017.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Wire.h>


const int NUM_READ = 64;  // количество усреднений для средних арифм. фильтров

#define V_HIGH 12.6
#define V_LOW 11.2
#define DIFF (V_HIGH-V_LOW)
#define TRH 0.1 

Adafruit_MCP23017 mcp;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

sensors_event_t event; 
//float imu_x;
float imu_y;
float imu_z;

int key = 0;

int imu_x;
//int imu_y;
//int imu_z;

byte *x;
byte *y;
byte *z;

int a = 0;
int b = 0;
int res = 0;

bool POWER = false;
int POWER_1 = 4;
int POWER_2 = 5;

int main_pin;
int main_led;

int other_pin;
int other_led;

float POWER_SIGNAL_1 = 0.0;
float POWER_SIGNAL_2 = 0.0;

float border_1 = 0.75;
float border_2 = 0.5;
float border_3 = 0.25;
float border_4 = 0.15;

int value = 0;
bool KONDO_POWER = 0;  
float zaryad = 0.0;   

bool POWER_1_STATUS = true;
bool POWER_2_STATUS = false; 

float k = 28.0/1024.0;//*1.024;

typedef union {
  float floattingpoint;
  byte binary[4];
}binaryFloat;

binaryFloat xi, yi, zi;

struct power_source
{
    int pin;
    int led;
    int apin;
    float v_signal = 0.0;
};


float midArifm(int pin) {
  float sum = 0;                     
  for (int i = 0; i < NUM_READ; i++) 
    sum += analogRead(pin);;                
  return (sum / NUM_READ);
  } 

int retbut(Adafruit_MCP23017 &mcp){
  int c = 0;
  while(1){
          if (!mcp.digitalRead(3)){
              c = 0;
              break;
          }
          if (!mcp.digitalRead(4)){
              c = 1;
              break;
          }
          if (!mcp.digitalRead(5)){
              c = 2;
              break;
          }}
  
  return c;
  }

 
void setup() {

  Serial.begin(115200);
  
  Wire.begin();
  Wire.setClock(115200);
  mcp.begin(&Wire);
  
  mcp.pinMode(2, INPUT);
  mcp.pinMode(3, INPUT);
  mcp.pinMode(4, INPUT);
  mcp.pinMode(5, INPUT);
  mcp.pullUp(2, HIGH);
  mcp.pullUp(3, HIGH);
  mcp.pullUp(4, HIGH);
  mcp.pullUp(5, HIGH);
  mcp.pullUp(6, HIGH);
  mcp.pinMode(0, OUTPUT);
  mcp.pinMode(1, OUTPUT);
  mcp.pinMode(7, OUTPUT);
  mcp.pinMode(8, OUTPUT);
  mcp.pinMode(9, OUTPUT);
  mcp.pinMode(10, OUTPUT);
  mcp.pinMode(11, OUTPUT);
  mcp.pinMode(12, OUTPUT);
  mcp.pinMode(13, OUTPUT);
  mcp.pinMode(14, OUTPUT);
  mcp.pinMode(15, OUTPUT);
  
  pinMode(POWER_1, OUTPUT);
  pinMode(POWER_2, OUTPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(22, OUTPUT);
  pinMode(25, OUTPUT);

  //setting all led OFF
  mcp.digitalWrite(7, LOW);
  mcp.digitalWrite(8 , LOW);
  mcp.digitalWrite(9 , LOW);
  mcp.digitalWrite(10 , LOW);
  mcp.digitalWrite(11 , LOW);
  mcp.digitalWrite(12 , LOW);
  mcp.digitalWrite(13 , LOW);
  mcp.digitalWrite(14 , LOW);
  mcp.digitalWrite(15 , LOW);

}

void loop() {

  mcp.digitalWrite(15 , LOW);
  bno.getEvent(&event);
  
 
  //imu_x = round(event.orientation.x)*1000;
  //float imu_x_test = event.orientation.x;
  //imu_y = round(event.orientation.y);
  //imu_z = round(event.orientation.z);
  
  //imu_x = event.orientation.x;
  imu_x = event.orientation.y;
  imu_y = event.orientation.y;
  imu_z = event.orientation.z;
  
  //*x = imu_x;
  //*y = imu_y;
  //*z = imu_z;
  
  //xi.floattingpoint = imu_x;
  //yi.floattingpoint = imu_y;
  //zi.floattingpoint = imu_z;

  
 
  

  
  
  if (Serial.available() > 0) {
    key = Serial.read();

    //cases 
    // 1 - init imu
    // 2 - read imu data and send to atom
    // 3 - ack for buttoms 

    
    switch(key){
      case 2:
        if(!bno.begin()){
         /* There was a problem detecting the BNO055 ... check your connections */
          Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
          while(1);
        }
      
        delay(500);
        
        bno.setExtCrystalUse(true);
        
        break;
        
      case 3:
        //byte *b = (byte *) & imu_x;
        //Serial.print(imu_x);
        
        
        //Serial.write(x, 2);
        //Serial.write(y, 2);
        
        //Serial.print(imu_x_test);
        //Serial.write(xi.binary, 4);
        //Serial.write(b, 4);
        
        //Serial.write(b, 2);
        //Serial.println(imu_x);
        //Serial.println(imu_x_test);
        //Serial.write(yi.binary, 4);
        //Serial.write(zi.binary, 4);
        //Serial.write(imu_z);
        
        //Serial.print(imu_x, 1);
        //Serial.print(imu_y, 1);
        //Serial.print(imu_z, 1);
        
        //Serial.print(imu_x);
        //Serial.print(' ');
        //Serial.print(imu_y);
        //Serial.print(' ');
        //Serial.println(imu_z);
        Serial.println(event.orientation.x, 6);
        Serial.println(event.orientation.y, 6);
        Serial.println(event.orientation.z, 6);
               
        break;
        
      case 1:
      
      mcp.digitalWrite(13, HIGH);
      mcp.digitalWrite(14, HIGH);
      mcp.digitalWrite(15, HIGH);

      a = retbut(mcp);
      delay(500);
      b = retbut(mcp);
      
      res = 3*a+b+1;
      while(1){
      if ((!mcp.digitalRead(2))&& (res >0) && (res<10)){
        Serial.println(res); 
        mcp.digitalWrite(13, LOW);
        mcp.digitalWrite(14, LOW);
        mcp.digitalWrite(15, LOW);
        break;   
          }
      else{
        //mcp.digitalWrite(13, LOW);
        //mcp.digitalWrite(13, HIGH);
        //a = retbut(mcp);
        //b = retbut(mcp);
        //res = 3*a+b+1;
        delay(1);}}
          
      
      break;

              
           
        
      default:
        delay(1);
      }
    }

  
  
  //digitalWrite(4, HIGH);
  static power_source power_1;
  static power_source power_2;
  
  power_1.pin = POWER_1;
  power_1.led = 0;
  power_1.apin = 3;
  power_2.pin = POWER_2;
  power_2.led = 1;
  power_2.apin = 4;

  main_pin = power_1.pin;
  main_led = power_1.led;

  other_led = power_2.pin;
  other_led = power_2.led;
  
  /*
  if ((main_pin == power_1.pin)&&(analogRead(power_2.apin)*k - analogRead(power_1.apin)*k > 0.3)){ 
    
    main_pin = power_2.pin;
    main_led = power_2.led;

    other_led = power_1.pin;
    other_led = power_1.led;;
    }
  if ((main_pin == power_2.pin)&&(analogRead(power_1.apin)*k - analogRead(power_2.apin)*k > 0.3)){
   
    main_pin = power_1.pin;
    main_led = power_1.led;

    other_led = power_2.pin;
    other_led = power_2.led; 
    }

  
  //Serial.println(power.pin);
  */
  if (!mcp.digitalRead(6))
    POWER = !POWER;
  if (POWER)
    mcp.digitalWrite(12, HIGH);
  else
    mcp.digitalWrite(12, LOW);
    
  //power 
  //digitalWrite(power.pin, HIGH);
  
  if (POWER){
    digitalWrite(main_pin, HIGH);
    digitalWrite(other_pin, LOW);
    mcp.digitalWrite(main_led, HIGH);
    mcp.digitalWrite(other_led, LOW);
  }
  else {
    digitalWrite(main_pin, LOW);
    mcp.digitalWrite(main_led, LOW);
  }
 /*
  float max;
  power_1.v_signal = midArifm(3);
  power_2.v_signal = midArifm(4);
 

  if ((k*power_1.v_signal > TRH)||(k*power_2.v_signal > TRH)){
    if (k*power_1.v_signal > k*power_2.v_signal){
      max = k*power_1.v_signal;}
      else
      {max = k*power_2.v_signal;}
    zaryad = (DIFF - (V_HIGH - max))/DIFF;
  }else{
    zaryad = 0.0;
  }
  
  if (zaryad > border_1){
    mcp.digitalWrite(11, HIGH);
    mcp.digitalWrite(10, HIGH);
    mcp.digitalWrite(9, HIGH);
    mcp.digitalWrite(8, HIGH);
  }else if (zaryad > border_2){
    mcp.digitalWrite(11, LOW);
    mcp.digitalWrite(10, HIGH);
    mcp.digitalWrite(9, HIGH);
    mcp.digitalWrite(8, HIGH);
  }else if (zaryad > border_3){
    mcp.digitalWrite(11, LOW);
    mcp.digitalWrite(10, LOW);
    mcp.digitalWrite(9, HIGH);
    mcp.digitalWrite(8, HIGH);
  }else if (zaryad > border_4){
    mcp.digitalWrite(11, LOW);
    mcp.digitalWrite(10, LOW);
    mcp.digitalWrite(9, LOW);
    mcp.digitalWrite(8, HIGH);
  }else {
    mcp.digitalWrite(8, LOW);
    mcp.digitalWrite(9, LOW);
    mcp.digitalWrite(10, LOW);
    mcp.digitalWrite(11, LOW);
  }
  
  zaryad = 0.0;  
  
  power_1.v_signal = 0.0;
  power_2.v_signal = 0.0;
  */
  //delay(200);
 
}
