

#include <ArduCAM.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "memorysaver.h"
/*  FOR BLUETOOTH */

#define SR04_TRIG_PIN 8
#define SR04_ECHO_PIN 5 

const int MaxDistanceTimeout = 20000;


#include <SoftwareSerial.h>
SoftwareSerial BT(A0, A1);  // Pin A0 directly connected to BT TX
                            // Pin A1 directly connected to BT RX 
                            // Keeps TX and RX (1 and 0) available for Serial Monitor


#if !(defined OV2640_MINI_2MP)
  #error Please select the hardware platform and camera module in the ../libraries/ArduCAM/memorysaver.h file
#endif
#define SD_CS 9
const int SPI_CS = 7;
#if defined (OV2640_MINI_2MP)
  ArduCAM myCAM( OV2640, SPI_CS );
#endif

//For New Files
char str[8];
byte buf[256];
static int i = 0;
static int k = 0;
uint8_t temp = 0,temp_last=0;
uint32_t length = 0;
bool is_header = false;
File outFile;

volatile bool interruptFlag = false;
volatile bool motionDetector = false;
volatile int initial_distance;

void setup(){
    uint8_t vid, pid;
    uint8_t temp;
    Wire.begin();
    BT.begin(9600);
    Serial.begin(115200);
    Serial.println(F("ArduCAM Start!"));
    //set the CS as an output:
    pinMode(SPI_CS,OUTPUT);
    digitalWrite(SPI_CS, HIGH);
  pinMode(SR04_TRIG_PIN, OUTPUT);
  pinMode(SR04_ECHO_PIN, INPUT);
    
    DDRD &= 0xF7; //set D3 as an input (Button)
    DDRD |= 0x10; //set D4 as an output (LED)
    PORTD |= 0x08;//INPUT PULLUP
    
    // initialize SPI:
    SPI.begin();
      
    //Reset the CPLD
    myCAM.write_reg(0x07, 0x80);
    delay(100);
    myCAM.write_reg(0x07, 0x00);
    delay(100);
      
    while(1){
      //Check if the ArduCAM SPI bus is OK
      myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
      temp = myCAM.read_reg(ARDUCHIP_TEST1);
      
      if (temp != 0x55){
        Serial.println(F("SPI interface Error!"));
        delay(1000);continue;
      }else{
        Serial.println(F("SPI interface OK."));break;
      }
    }
    //Initialize SD Card
    while(!SD.begin(SD_CS)){
      Serial.println(F("SD Card Error!"));delay(1000);
    }
    Serial.println(F("SD Card detected."));
    
    #if defined (OV2640_MINI_2MP)
      while(1){
        //Check if the camera module type is OV2640
        myCAM.wrSensorReg8_8(0xff, 0x01);
        myCAM.rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
        myCAM.rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
        if ((vid != 0x26 ) && (( pid != 0x41 ) || ( pid != 0x42 ))){
          Serial.println(F("Can't find OV2640 module!"));
          delay(1000);continue;
        }
        else{
          Serial.println(F("OV2640 detected."));break;
        } 
      }
    #endif
    
    myCAM.set_format(JPEG);
    myCAM.InitCAM();
    #if defined (OV2640_MINI_2MP)
      myCAM.OV2640_set_JPEG_size(OV2640_800x600);
    #endif
    delay(1000);

    initial_distance = distanceInCm() -5;
    Serial.println("initial distance is"); Serial.println(initial_distance);
    
    //external interrupt setup
    cli();
    EICRA |= 0b00000000; //low level
    EIMSK |= 0x02; //enable external interrupt 1
    EIFR |=0x02; //clear interrupt
    sei();
}

void loop(){
//initial_distance = distanceInCm();
//Serial.println("initial distance is"); Serial.println(initial_distance);

if(interruptFlag){
  Serial.println("System Active");
  PORTD |= 0x10; //turn on LED
  motion_detection();
  Serial.println(motionDetector);
  
  if(motionDetector){
  cli();
  //Take picture and Save
    //Flush the FIFO
  myCAM.flush_fifo();
  //Clear the capture done flag
  myCAM.clear_fifo_flag();
  //Start capture
  myCAM.start_capture();
  Serial.println(F("start Capture"));
  while(!myCAM.get_bit(ARDUCHIP_TRIG , CAP_DONE_MASK));
  Serial.println(F("Capture Done."));  
  length = myCAM.read_fifo_length();
//  Serial.print(F("The fifo length is :"));
//  Serial.println(length, DEC);
  if (length >= MAX_FIFO_SIZE) //384K
  {
    Serial.println(F("Over size."));
    return ;
  }
  if (length == 0 ) //0 kb
  {
    Serial.println(F("Size is 0."));
    return ;
  }
  //Construct a file name
  k = k + 1;
  itoa(k, str, 10);
  strcat(str, ".jpg");
  
  //Open the new file
  outFile = SD.open(str, O_WRITE | O_CREAT | O_TRUNC);
  if(!outFile){
    Serial.println(F("File open failed"));
    return;
  }
  myCAM.CS_LOW();
  myCAM.set_fifo_burst();
  while ( length-- )
  {
    temp_last = temp;
    temp =  SPI.transfer(0x00);
    //Read JPEG data from FIFO
    if ( (temp == 0xD9) && (temp_last == 0xFF) ) //If find the end ,break while,
    {
      buf[i++] = temp;  //save the last  0XD9     
      //Write the remain bytes in the buffer
      myCAM.CS_HIGH();
      outFile.write(buf, i);    
      //Close the file
      outFile.close();
      Serial.println(F("Image save OK."));
      is_header = false;
      i = 0;
    }  
    if (is_header == true)
    { 
      //Write image data to buffer if not full
      if (i < 256)
      buf[i++] = temp;
      else
      {
        //Write 256 bytes image data to file
        myCAM.CS_HIGH();
        outFile.write(buf, 256);
        i = 0;
        buf[i++] = temp;
        myCAM.CS_LOW();
        myCAM.set_fifo_burst();
      }        
    }
    else if ((temp == 0xD8) & (temp_last == 0xFF))
    {
      is_header = true;
      buf[i++] = temp_last;
      buf[i++] = temp;   
    } 
  }
  delay(5000); 
  sei();
  }
 }

  
else{
    PORTD &= 0xEF; //turn off LED
    Serial.println("System Disabled");
  }
}

//****************************************************************
void motion_detection(){
  int new_distance = distanceInCm();
//  Serial.println("new distance is"); Serial.println(new_distance);

  if(initial_distance > new_distance){
    motionDetector = true;
  }
  else{
    motionDetector = false;
  }
  
}
//****************************************************************
int distanceInCm(){
  //sending the signal, starting with LOW for a clean signal
  digitalWrite(SR04_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(SR04_TRIG_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(SR04_TRIG_PIN, LOW);
  int distance = pulseIn(SR04_ECHO_PIN, HIGH, MaxDistanceTimeout) / 29 / 2; 
  // timeout after 9000 microseconds (150 cm)
  if (distance == 0) distance = MaxDistanceTimeout / 29 / 2; 
  // if no echo, assume object is at farthest distance
  return distance;
  // if pulse
} //distanceInCm()
//****************************************************************
ISR(INT1_vect){
interruptFlag = !interruptFlag;
EIFR |= 0x02; //clear flag
  //Serial.println("Interrupt");
}
