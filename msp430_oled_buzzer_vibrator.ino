/*
    MPU6050 Triple Axis Gyroscope & Accelerometer. Pitch & Roll Accelerometer Example.
    Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-zyroskop-i-akcelerometr-mpu6050.html
    GIT: https://github.com/jarzebski/Arduino-MPU6050
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/

#include <Wire.h>
#include "Font.h"
#include <string.h>
#include "images.h"
#include <MPU6050.h>

MPU6050 mpu;

// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;
#define OLED_Write_Address 0x3C
const int pot_input = A4;
bool d1 = HIGH;
bool d2 = LOW;

void OLED_Data(char *DATA) /* Function for sending data to OLED */
{
  int len = strlen(DATA);
  for (int g=0; g<len; g++)
  {    
    for (int index=0; index<5; index++)
    {     
      Wire.beginTransmission(OLED_Write_Address); /* Begin transmission to slave device */
    /* Queue data to be transmitted */
      Wire.write(0x40); /* For Data Transmission, C = 0 and D/C = 1 */ 
      Wire.write(ASCII[DATA[g] - 0x20][index]); 
      Wire.endTransmission(); /* Transmit the queued bytes and end transmission to slave device */ 
    }    
  }
}

void OLED_Command(char DATA) /* Function for sending command to OLED */
{
  Wire.beginTransmission(OLED_Write_Address); /* Begin transmission to slave device */
  /* Queue data to be transmitted */
  Wire.write(0x00); /* For Data Transmission, C = 0 and D/C = 0 */ 
  Wire.write(DATA); 
  Wire.endTransmission(); /* Transmit the queued bytes and end transmission to slave device */
}

void OLED_Command(int INTEGER) /* Function for sending command to OLED */
{
  Wire.beginTransmission(OLED_Write_Address); /* Begin transmission to slave device */
  /* Queue data to be transmitted */
  Wire.write(0x00); /* For Data Transmission, C = 0 and D/C = 0 */ 
  Wire.write(INTEGER); 
  Wire.endTransmission(); /* Transmit the queued bytes and end transmission to slave device */
}

void OLED_clear(void) /* Function for clearing OLED */   
{
  OLED_setXY(0x00, 0x7F, 0x00, 0x07); /* Column Start Address 0, Column End Address 127, Page Start Address 0, Page End Address 7  */
  for (int k=0; k<=1023; k++)
  {
    Wire.beginTransmission(OLED_Write_Address); /* Begin transmission to slave device */
  /* Queue data to be transmitted */
    Wire.write(0x40); /* For Data Transmission, C = 0 and D/C = 1 */ 
    Wire.write(0x00); 
    Wire.endTransmission(); /* Transmit the queued bytes and end transmission to slave device */  
  }  
}

void OLED_setXY(char col_start, char col_end, char page_start, char page_end) /* Function for setting cursor for writing data */  
{
  Wire.beginTransmission(OLED_Write_Address); /* Begin transmission to slave device */
  /* Queue data to be transmitted */
  Wire.write(0x00); /* For Data Transmission, C = 0 and D/C = 0 */
  Wire.write(0x21); /* Set Column Start and End Address */ 
  Wire.write(col_start); /* Column Start Address col_start */
  Wire.write(col_end); /* Column End Address col_end */
  Wire.write(0x22); /* Set Page Start and End Address */ 
  Wire.write(page_start); /* Page Start Address page_start */
  Wire.write(page_end); /* Page End Address page_end */
  Wire.endTransmission(); /* Transmit the queued bytes and end transmission to slave device */
}

void OLED_init(void) /* Function for initializing OLED */
{
  OLED_Command(0xAE); /* Entire Display OFF */
  OLED_Command(0xD5); /* Set Display Clock Divide Ratio and Oscillator Frequency */
  OLED_Command(0x80); /* Default Setting for Display Clock Divide Ratio and Oscillator Frequency that is recommended */
  OLED_Command(0xA8); /* Set Multiplex Ratio */
  OLED_Command(0x3F); /* 64 COM lines */
  OLED_Command(0xD3); /* Set display offset */
  OLED_Command(0x00); /* 0 offset */
  OLED_Command(0x40); /* Set first line as the start line of the display */
  OLED_Command(0x8D); /* Charge pump */
  OLED_Command(0x14); /* Enable charge dump during display on */
  OLED_Command(0x20); /* Set memory addressing mode */
  OLED_Command(0x00); /* Horizontal addressing mode */
  OLED_Command(0xA1); /* Set segment remap with column address 127 mapped to segment 0 */
  OLED_Command(0xC8); /* Set com output scan direction, scan from com63 to com 0 */
  OLED_Command(0xDA); /* Set com pins hardware configuration */
  OLED_Command(0x12); /* Alternative com pin configuration, disable com left/right remap */
  OLED_Command(0x81); /* Set contrast control */
  OLED_Command(0x80); /* Set Contrast to 128 */
  OLED_Command(0xD9); /* Set pre-charge period */
  OLED_Command(0xF1); /* Phase 1 period of 15 DCLK, Phase 2 period of 1 DCLK */
  OLED_Command(0xDB); /* Set Vcomh deselect level */
  OLED_Command(0x20); /* Vcomh deselect level ~ 0.77 Vcc */
  OLED_Command(0xA4); /* Entire display ON, resume to RAM content display */
  OLED_Command(0xA6); /* Set Display in Normal Mode, 1 = ON, 0 = OFF */
  OLED_Command(0x2E); /* Deactivate scroll */
  OLED_Command(0xAF); /* Display on in normal mode */
}

void OLED_image(const unsigned char *image_data)   /* Function for sending image data to OLED */
{
  OLED_setXY(0x00, 0x7F, 0x00, 0x07);    
  for (int k=0; k<=1023; k++)
  {    
    Wire.beginTransmission(OLED_Write_Address); /* Begin transmission to slave device */
  /* Queue data to be transmitted */
    Wire.write(0x40); /* For Data Transmission, C = 0 and D/C = 1 */
    Wire.write(image_data[k]);
    Wire.endTransmission(); /* Transmit the queued bytes and end transmission to slave device */ 
  }   
}

void setup() 
{
  Serial.begin(9600);

  Serial.print("Initialize MPU6050");

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(50);
  }
  //checkSettings();
  pinMode(12, OUTPUT);  /* Motor control pin 1 */
  pinMode(13, OUTPUT);  /* Motor control pin 2 */
  //pinMode(14, OUTPUT);  /* PWM pin for Speed Control */
  pinMode(5, INPUT_PULLUP);  /* Interrupt pin for direction control */
//  attachInterrupt(5, motor, FALLING);  /* Interrupt on falling edge on pin 5 */
Wire.begin(); /* Initiate wire library and join I2C bus as a master */
  OLED_init(); /* Initialize OLED */
  delay(10);  
  OLED_clear(); /* Clear OLED */
  delay(10);
  OLED_image(Launchpad_Logo);
  delay(2000);
  OLED_clear();
  delay(20);
  
  OLED_setXY(0x31, 0x7F, 0x03, 0x02);
  OLED_Data("Hello ");
  OLED_setXY(0x36, 0x7F, 0x04, 0x03);
  OLED_Data("SAFE DRIVE");
  OLED_setXY(0x00, 0x7F, 0x00, 0x07);
  delay(2000); 
  OLED_clear();
  delay(20);
}

void loop()
{
  // Read normalized values
  Vector norm = mpu.readNormalizeGyro();
Vector normAccel = mpu.readNormalizeAccel();
  // Calculate Pitch, Roll and Yaw
  pitch =  norm.YAxis * timeStep;
  roll = norm.XAxis * timeStep;
  yaw =  norm.ZAxis * timeStep;

  // Output raw
  Serial.print(" Pitch = ");
  Serial.print(pitch);
  Serial.print(" Roll = ");
  Serial.print(roll);  
  Serial.print(" Yaw = ");
  Serial.print(yaw);
  
  Serial.print(" Xnorm = ");
  Serial.print(normAccel.XAxis);
  Serial.print(" Ynorm = ");
  Serial.print(normAccel.YAxis);
  Serial.print(" Znorm = ");
  Serial.println(normAccel.ZAxis);
  //delay(100);
  int pwm_adc;
 // pwm_adc = analogRead(pot_input); /* Input from Potentiometer for speed control */
  //Serial.print(" PWM = ");
  //Serial.print(pwm_adc);  
  digitalWrite(12,d1);
  digitalWrite(13,d2);
      if(pitch >= 0.45  )
          {
               analogWrite(14, 1023 / 4); 
                OLED_setXY(0x31, 0x7F, 0x03, 0x02);
                OLED_Data("Alarm is ");
                OLED_setXY(0x36, 0x7F, 0x04, 0x03);
                OLED_Data("ON");
                OLED_setXY(0x00, 0x7F, 0x00, 0x07); 
                OLED_clear();
                delay(20);
          }
      else if(pitch <= -0.45  )
          {
                analogWrite(14, 1023 / 4);
                OLED_setXY(0x31, 0x7F, 0x03, 0x02);
                OLED_Data("Alarm is ");
                OLED_setXY(0x36, 0x7F, 0x04, 0x03);
                OLED_Data("ON");
                OLED_setXY(0x00, 0x7F, 0x00, 0x07); 
                OLED_clear();
                delay(20);
          }
       else
          {
                analogWrite(14, 0);
                OLED_setXY(0x31, 0x7F, 0x03, 0x02);
                OLED_Data("Alarm is ");
                OLED_setXY(0x36, 0x7F, 0x04, 0x03);
                OLED_Data("OFF");
                OLED_setXY(0x00, 0x7F, 0x00, 0x07); 
                OLED_clear();
                delay(20);
           }
        if(roll >= 0.45  )
            {
                analogWrite(14, 1023 / 4); 
                OLED_setXY(0x31, 0x7F, 0x03, 0x02);
                OLED_Data("Alarm is ");
                OLED_setXY(0x36, 0x7F, 0x04, 0x03);
                OLED_Data("ON");
                OLED_setXY(0x00, 0x7F, 0x00, 0x07); 
                OLED_clear();
                delay(20);
            }
        else if(roll <= -0.45  )
            {
                analogWrite(14, 1023 / 4);
                OLED_setXY(0x31, 0x7F, 0x03, 0x02);
                OLED_Data("Alarm is ");
                OLED_setXY(0x36, 0x7F, 0x04, 0x03);
                OLED_Data("ON");
                OLED_setXY(0x00, 0x7F, 0x00, 0x07); 
                OLED_clear();
                delay(20);
            }
        if(yaw >= 0.70  )
            {
                analogWrite(14, 1023 / 4); 
                OLED_setXY(0x31, 0x7F, 0x03, 0x02);
                OLED_Data("Alarm is ");
                OLED_setXY(0x36, 0x7F, 0x04, 0x03);
                OLED_Data("ON");
                OLED_setXY(0x00, 0x7F, 0x00, 0x07); 
                OLED_clear();
                delay(20);
            }
        else if(yaw <= -0.70  )
            {
                analogWrite(14, 1023 / 4);
                OLED_setXY(0x31, 0x7F, 0x03, 0x02);
                OLED_Data("Alarm is ");
                OLED_setXY(0x36, 0x7F, 0x04, 0x03);
                OLED_Data("ON");
                OLED_setXY(0x00, 0x7F, 0x00, 0x07); 
                OLED_clear();
                delay(20);
            }
}
//void motor()
//{
  //d1 = !d1;
 // d2 = !d2;
  //for(int i = 0; i<10000; i++)
  //for(int j = 0; j<10; j++);
//}

