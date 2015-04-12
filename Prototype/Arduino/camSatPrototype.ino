/*this code is developed by Mahmoud Elmohr, Abdallah Mohammed, Ahmed El-Tawill for ( CAN-SAT II ) project 
which was held by Planterium Science center Bibliotheca Alexandrina and supported by UNISEC Egypt*/
/*this code demonstrates data aquistion for a gyroscope,accelerometer,humidity and temperature sensors 
and send the data by Xbee to a ground station, also to control parachut through servo motor */
/* to be mentioned we faced problem with GPS in hardware that's why we excluded it 
and also faced problems with presuure-altimeter sensor due to lack of analog pins
but both code are written exclusivley in seperate sketches*/



////declarations////

  short temperature;

/// servo for parachut ///
#include <Servo.h>                                          //servo library
Servo parachutte;                                           //parachutte is controlled by servo
char reading;                                               //readings comming from Xbee for parachutte deployement


///gyro - acc ///
float x_ac, y_ac, z_ac;                                      //accelerations from accelerometer
float x_ang,y_ang;                                           //final angles 
int xzero_ac, yzero_ac,zzero_ac;                             //initial refrences
float g=9.81;                                                //gravity
unsigned long now;                                           //time for ground station

///humidity ///
/*#include "DHT.h"                                             //DHT sensor library
HT dht;                                                     
 int humidity;              
 int temperature_h;*/

///pressure///
# define BMP085_ADDRESS 0x77  // I2C address of BMP085
# include <wire.h>
// Read 1 byte from the BMP085 at 'address'
char bmp085Read(unsigned char address)
{
  unsigned char data;
  
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  
  Wire.requestFrom(BMP085_ADDRESS, 1);
  while(!Wire.available())
    ;
    
  return Wire.read();
}

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp085ReadInt(unsigned char address)
{
  unsigned char msb, lsb;
  
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  
  Wire.requestFrom(BMP085_ADDRESS, 2);
  while(Wire.available()<2)
    ;
  msb = Wire.read();
  lsb = Wire.read();
  
  return (int) msb<<8 | lsb;
}



////////////////////


// This is the top of the program
#include <Wire.h>

#define BMP085_ADDRESS 0x77  // I2C address of BMP085



const unsigned char OSS = 0;  // Oversampling Setting

// Calibration values
int ac1;
int ac2; 
int ac3; 
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1; 
int b2;
int mb;
int mc;
int md;

// b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...)
// so ...Temperature(...) must be called before ...Pressure(...).
long b5; 

short temperature_p;
float pressure;
float pressure_filtered=101520;

// Use these for altitude conversions
 float po=0;    // Pressure at sea level (Pa)
float altitud;

void setup()
{
  pinMode(13,OUTPUT);
   ///parachut servo ///
   parachutte.attach(3);


  ///gyro - acc ///
  Serial.begin(9600);                                       //serial communication for xbee   
  xzero_ac=analogRead(A0);
  yzero_ac=analogRead(A1);
  zzero_ac=analogRead(A2)-65;                               //-65 to remove the effect of gravity assuming that initialization is done while z axis is perpendicular to ground
 
  
  ///humidity ///
 /* dht.setup(2); */                //humidity sensor initialization
  
  /// pressure ///
  Wire.begin();
  bmp085Calibration();
}


// pressure//
// Stores all of the bmp085's calibration values into global variables
// Calibration values are required to calculate temp and pressure
// This function should be called at the beginning of the program
void bmp085Calibration()
{
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
}


///////////////////
// Read the uncompensated temperature value
unsigned int bmp085ReadUT()
{
  unsigned int ut;
  
  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();
  
  // Wait at least 4.5ms
  delay(5);
  
  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  return ut;
}

// Read the uncompensated pressure value
unsigned long bmp085ReadUP()
{
  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;
  
  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS<<6));
  Wire.endTransmission();
  
  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3<<OSS));
  
  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF6);
  Wire.endTransmission();
  Wire.requestFrom(BMP085_ADDRESS, 3);
  
  // Wait for data to become available
  while(Wire.available() < 3)
    ;
  msb = Wire.read();
  lsb = Wire.read();
  xlsb = Wire.read();
  
  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);
  
  return up;
}


////////////////////

// Calculate temperature given ut.
// Value returned will be in units of 0.1 deg C
short bmp085GetTemperature(unsigned int ut)
{
  long x1, x2;
  
  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  return ((b5 + 8)>>4);  
}

// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
long bmp085GetPressure(unsigned long up)
{
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;
  
  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;
  
  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;
  
  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;
    
  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;
  
  return p;
}
boolean flag;



void loop()
{
  

  now = millis();                                           //get the time now since the arduino is on

  ///gyro - acc ///
  x_ac = (analogRead(A0)-xzero_ac)*g/67.518;      //real reading = reading-refrence , and the 61.44 is to convert readings into acceleration in m/s^2
                                                  //as 5v gives 1023 then 300mv gives 1023*.3/5 which is 61.44 and this value of reading equals 1 g whic is 9.8 m/s^2
  
  y_ac = (analogRead(A1)-yzero_ac)*g/67.518;      //same for x 
  z_ac = (analogRead(A2)-zzero_ac)*g/67.518;      //same for x 
 
 
   x_ang = -1*atan2(x_ac, sqrt(y_ac*y_ac + z_ac*z_ac))*180.0/3.14;      //this equation is to caculae anges from accelerations , -1 due to position of accelerometer and gyroscope to make thier outputs in phase
    y_ang = -1*atan2(y_ac, sqrt(x_ac*x_ac + z_ac*z_ac))*180.0/3.14;     //this equation is to caculae anges from accelerations , -1 due to position of accelerometer and gyroscope to make thier outputs in phase
  ////////////////////////////////////////  
     if(x_ang > 90)
            x_ang=180-x_ang;               //to keep angles from accelerometer between -90 and 90 to avoid problems
     else if(x_ang < -90)
            x_ang=-180-x_ang;
     if(y_ang > 90)
            y_ang=180-y_ang;
     else if(y_ang < -90)
            y_ang=-180-y_ang;
 ///////////////////////////////////////////////////////
  if (x_ang>180)                                                         //to keep final angles between -180 and 180 
        x_ang = -360+ x_ang ;
  else if (x_ang<-180)
        x_ang = 360+ x_ang ;
  if (y_ang>180)
        y_ang = -360+ y_ang ;
  else if (y_ang<-180)
        y_ang = 360+ y_ang ;
  /////////////////////////
  if ((x_ang<=1) && (x_ang>=-1))
 {
   x_ang=0;                                      // this to ellimnate small errors which may have effects when integerated
 } 
if ((y_ang<=1) && (y_ang>=-1))
 {
   y_ang=0;                                      // this to ellimnate small errors which may have effects when integerated
 }  
 
 
   ////humidity /////////////////

 /*int humidity = dht.getHumidity();               //to get humidity
 int temperature_h= dht.getTemperature();  */       //to get temperature
  
  
  
  
  //// pressure ////
  
  /// excuted once to calculate aproximate po ///
  if (flag==LOW)
      {
  
        for( int i=0; i<100 ;i++)
       {
        temperature_p = 0.1*bmp085GetTemperature(bmp085ReadUT());
        pressure = bmp085GetPressure(bmp085ReadUP());
        pressure_filtered += .5 * (pressure- pressure_filtered);

       }
        po=pressure_filtered;
        flag=HIGH;
      }
      
  temperature_p = 0.1*bmp085GetTemperature(bmp085ReadUT());
  pressure = bmp085GetPressure(bmp085ReadUP());
  pressure_filtered += .5 * (pressure- pressure_filtered);

  
  ///  temperature combination ///
  temperature =  temperature_p ;
  
  
  //////ALTITUDE//
  // Add this into loop(), after you've calculated the pressure
  altitud = 44330 * (1 - pow(( pressure_filtered/po), 0.190295));
 
  
  ////g  (print) /////////////////
  
 /* Serial.print("Time           ");       
  Serial.println(now, DEC);*/
  Serial.print("X_angle        ");       
  Serial.println(x_ang, DEC);    
  Serial.print("Y_angle        ");       
  Serial.println(y_ang, DEC);  
  /*Serial.print("Altitude       ");
  Serial.println(altitud);*/
  Serial.print("Presssure      ");
  Serial.println(pressure);
  Serial.print("Temperature  ");
  Serial.println(temperature);

  /*Serial.println("Humidity       33");*/
  Serial.println(" ");


  
  
  /// parachute ///
  reading=Serial.read();
     if ((reading=='p') || (reading =='P'))
  {
          parachutte.write(90);
          delay(15);
          parachutte.write(0);
          delay(15);  
          digitalWrite(13,HIGH);
      }
       if ((reading=='r') || (reading =='R'))
  {
     xzero_ac=analogRead(A0);
  yzero_ac=analogRead(A1);
  zzero_ac=analogRead(A2)-65;     //-65 to remove the effect of gravity assuming that initialization is done while z axis is perpendicular to ground
 flag=LOW;
          
      }

 /// put yor delays here (salah) ///
 delay(1000);

}




