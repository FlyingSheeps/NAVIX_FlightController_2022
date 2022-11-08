#include <Arduino.h>
#include "sbus.h"
#include <esp32-hal-ledc.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_BNO055.h"
#include "Adafruit_Sensor.h"
#include "VL53L1X.h"
//if you use this program, you need to download libraries above.

/*setup*/
struct datastr{

  //IMU properties
  float accX = 0.0F;
  float accY = 0.0F;
  float accZ = 0.0F;

  float gyroPitch = 0.0F;
  float gyroRoll = 0.0F;
  float gyroYaw = 0.0F;

  float pitch = 0.0F;
  float roll  = 0.0F;
  float yaw   = 0.0F;

  //tof properties
  float level = 0; 

  //objective angle[deg]
  float pitch_r = 0.0F;
  float roll_r  = 0.0F;
  float level_r   = 0.0F;

  //input[count]
  float pitch_u = 0.0;
  float roll_u = 0.0;;
  int ESC_u = 0;
  int gear = 0;

  //time[ms]
  int deltaT = 0;

  //booleans
  bool control_bool = 0;
  bool level_turn = 0;

};

datastr data;

//IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
sensors_event_t bnodata;

//ToF
VL53L1X tof;

//処理時間
unsigned long prev_time = 0;
//sbus
// channel, fail safe, and lost frames data
std::array<int16_t, bfs::SbusRx::NUM_CH()> sbus_data;
/* SbusRx object on Serial1 */
bfs::SbusRx sbus_rx(&Serial1);
#define SBUS_MAX 2048
#define SBUS_MIN 0


/*-------controlprocess--------*/
float p_control(float theta, float r, float gain);
float r_control(float phi,   float r, float gain);
float level_to_pitch_r(float level, float r, float gain);
uint16_t plev = 0;

//objective angle[deg]
#define PITCH_R_MIN -10.0
#define PITCH_R_MAX 10.0
#define ROLL_R_MIN -20.0
#define ROLL_R_MAX 20.0


//PWM properties
#define COUNT_LOW 2048 //0deg=1ms
#define COUNT_HIGH 4096 //180deg=2ms
#define PWM_FREQ 125 //T=8ms
#define DUTY_RESOLUTION 14 //sbus 11bit times 4



/*-------------setup------------------*/
void setup() {
  
  //init PWM
  ledcSetup(1, PWM_FREQ, DUTY_RESOLUTION);
  ledcAttachPin(32, 1);
  ledcSetup(2, PWM_FREQ, DUTY_RESOLUTION);
  ledcAttachPin(25, 2);
  ledcSetup(3, PWM_FREQ, DUTY_RESOLUTION);
  ledcAttachPin(26, 3);
  ledcSetup(4, PWM_FREQ, DUTY_RESOLUTION);
  ledcAttachPin(27, 4);
  //init throttle (if calibrated)
  ledcWrite(1, COUNT_LOW + 368 - 168);

  //init serial
  Serial.begin(115200);
  Serial.println("started...");
  delay(1000);
  //init i2c
  Wire.begin();
  Wire.setClock(400000);
  

  //init sbus
  sbus_rx.Begin(33,14);
  Serial.println("finished sbus setup");

  //init LED
  pinMode(17, OUTPUT);
  pinMode(16, OUTPUT);

  //check LED 
  digitalWrite(16, HIGH);
  digitalWrite(17, HIGH);
  delay(1000);
  digitalWrite(16, LOW);
  digitalWrite(17, LOW);
  delay(1000);

  //init IMU
  if (!bno.begin())
  {
    while(1){
      Serial.print("No BNO055 detected");
      delay(1000);
    }
  }
  //init ToF
  tof.setTimeout(500);
  if(!tof.init())
  {
    while(1){
      Serial.println("Failed to detect and initialize sensor!");
      delay(1000);
    }
  }
  tof.setDistanceMode(VL53L1X::Long);
  tof.setMeasurementTimingBudget(50000);
  tof.startContinuous(50);

}



/*----------control loop----------*/
void loop() {

  //time 
  data.deltaT = millis() - prev_time;
  prev_time = millis();

  //read IMU. please check axis of your own IMU
  bno.getEvent(&bnodata, Adafruit_BNO055::VECTOR_EULER);
  data.pitch =  -bnodata.orientation.z;
  data.roll  =  -bnodata.orientation.y;
  data.yaw   =   bnodata.orientation.x;
    

  //read tof
  plev = tof.read(0);
  data.level = 0.001*(float)plev*cos(data.pitch*3.14/180.0)*cos(data.roll*3.14/180.0);
  Serial.println(data.level);
  
  
  //read sbus
  if(sbus_rx.Read()){
    sbus_data = sbus_rx.ch();
    data.ESC_u = sbus_data[2] + COUNT_LOW;
    //boolean
    data.gear = sbus_data[4] + COUNT_LOW;
    data.control_bool = (sbus_data[6] > 1024) ? 1 : 0;
    data.level_turn = (sbus_data[7] > 1024) ? 1 : 0;
  }

  if(data.control_bool == 1 && data.level_turn == 1){
    //level turn
    digitalWrite(17, HIGH);
    digitalWrite(16, LOW);
    //feeding apriori object
    data.level_r = 2.75; //m
    data.roll_r = 10; //deg
      
    //control calc
    data.pitch_r = level_to_pitch_r(data.level,data.level_r,(float)sbus_data[5]/2048.0*10);
    data.pitch_u = p_control(data.pitch,data.pitch_r, (float)sbus_data[5]/2048*10);
    data.roll_u = r_control(data.roll,data.roll_r,(float)sbus_data[5]/2048*10);
    
    //sorvo PWM
    ledcWrite(2, constrain(data.pitch_u + (COUNT_LOW+COUNT_HIGH)/2,COUNT_LOW+368,COUNT_HIGH-368));
    ledcWrite(3, constrain(data.roll_u + (COUNT_LOW+COUNT_HIGH)/2,COUNT_LOW+368,COUNT_HIGH-368));
  }else if(data.control_bool == 1 && data.level_turn == 0){
    //assist
    digitalWrite(17, LOW);
    digitalWrite(16, HIGH);

    //sbus remap
    data.pitch_r = map(sbus_data[1], SBUS_MIN+368, SBUS_MAX-368, PITCH_R_MIN, PITCH_R_MAX);
    data.roll_r = map(sbus_data[3], SBUS_MIN+368, SBUS_MAX-368, ROLL_R_MIN, ROLL_R_MAX);
      
    //control calc
    data.pitch_u = p_control(data.pitch, data.pitch_r+5.0, (float)sbus_data[5]/2048*10);
    data.roll_u = r_control(data.roll, data.roll_r,(float)sbus_data[5]/2048*10);
    Serial.println(data.pitch_u);
    Serial.println(data.roll_u);
    //servo PWM
    ledcWrite(2, constrain(data.pitch_u + (COUNT_HIGH + COUNT_LOW)/2,COUNT_LOW+368,COUNT_HIGH-368));
    ledcWrite(3, constrain(data.roll_u + (COUNT_HIGH + COUNT_LOW)/2,COUNT_LOW+368,COUNT_HIGH-368));
  }else{
    //manual
    digitalWrite(17, LOW);
    digitalWrite(16, HIGH);
    data.pitch_u = sbus_data[1] + COUNT_LOW;
    data.roll_u = sbus_data[3] + COUNT_LOW;
    ledcWrite(2, data.pitch_u);
    ledcWrite(3, data.roll_u);
  }

    //common
    ledcWrite(4, data.gear);
    ledcWrite(1, data.ESC_u);

    
    //limiting delay (this is not a suitable way. please check metro or please use micros and while)
    if(data.deltaT < 10) { delay(10-data.deltaT); }
    

}



float p_control(float theta, float r, float gain){
  //Degree base
  float u = 0.0;
  
  //u = Cx
  u = -2*(theta - r)*2048/180.0;
  //linkrate×11bit/180deg
  return u;
}

float r_control(float phi, float r, float gain){
  //Degree base
  float u = 0.0;
  
  //u=Kx
  u = -2*(phi - r)*2048/180.0;
  //linkrate×11bit/180deg
  return u;
}

float level_to_pitch_r(float level, float r, float gain){
  //meter base
  float pitch_r = 0.0;
  //hight to objective pitch around pitch 5.0deg
  pitch_r = constrain(5.0-0.5/0.1*(level-r),PITCH_R_MIN,PITCH_R_MAX);
  //5cm->1deg
  return pitch_r;
}
