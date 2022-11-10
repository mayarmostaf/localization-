//need to change pins A B

//ros part 
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>
ros::NodeHandle n;
//ros imu
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);
//ros position 
geometry_msgs::Pose pose_xy_msg;
ros::Publisher pose ("pose",&pose_xy_msg);
////////////////////////////////////////////////////////////////////////////

//xy part 
float y;
float x;
float y_pre=0;
float x_pre=0;
float dx;
float dy;
int y_count_pre=0;
int x_count_pre=0;

////////////////////////////////////////////////////////////////////////////
float count_length;//???????intialize
//y encoder part
int y_A =2;
int y_B =3;
int y_count=0;
void y_ISR_A (void);
void y_ISR_B (void);
//x encoder part
int x_A =2;
int x_B =3;
int x_count=0;
void x_ISR_A(void);
void x_ISR_B (void);

/////////////////////////////////////////////////////////////////////////////

//imu part
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;// creating new imu from lib
#define OUTPUT_READABLE_YAWPITCHROLL
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yaw_rad;
//interrupt detection routine
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;}
//////////////////////////////////////////////////////////////////////////////

void setup() {
//ros setup
    n.getHardware()->setBaud(115200);
    n.getHardware()->setPort(&Serial1);
    n.initNode();
    Serial.begin(115200);
//ros imu
  n.advertise(imu_pub);
  imu_msg.header.frame_id = 0;
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 0.0;
//ros xy //?oreintation are angles and quarterian
    n.advertise(pose);
    pose_xy_msg.position.x=0.0;
    pose_xy_msg.position.y=0.0;
    pose_xy_msg.position.z=0.0;
    pose_xy_msg.orientation.x=0.0;
    pose_xy_msg.orientation.y=0.0;
    pose_xy_msg.orientation.z=0.0;
    pose_xy_msg.orientation.w=0.0;
    
    
    
  
//y encoder setup
pinMode(y_A,INPUT_PULLUP); 
pinMode(y_B,INPUT_PULLUP);
attachInterrupt(y_A,y_ISR_A,CHANGE);//NOTE IN STM WE CAN USE THE PIN DIRECTLY WITHOUT DIGITALPINTOINTERRUPT
attachInterrupt(y_B,y_ISR_B,CHANGE);
//x encoder setup
pinMode(x_A,INPUT_PULLUP); 
pinMode(x_B,INPUT_PULLUP);
attachInterrupt(x_A,x_ISR_A,CHANGE);//NOTE IN STM WE CAN USE THE PIN DIRECTLY WITHOUT DIGITALPINTOINTERRUPT
attachInterrupt(x_B,x_ISR_B,CHANGE);
/////////////////////////////////////////////////////////////////////////////

//imu setup
// join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    mpu.initialize();
// load and configure the DMP
    devStatus = mpu.dmpInitialize();
// supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
// make sure it worked (returns 0 if so)
    if (devStatus == 0) {
     // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
     // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);
        dmpReady = true;
     // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
                       }
}

void loop() {
////////////////////////////////////////////////////////////////////////////////
//imu loop
// if programming failed, don't try to do anything
    if (!dmpReady) return;

// read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      // Get the Latest packet 
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);   
            yaw_rad=ypr[0];
        #endif
                                                  }  

/////////////////////////////////////////////////////////////////////////////////
//call a method to calculate xy_co ordinares                                                   
         get_xy();
         
///////////////////////////////////////////////////////////////////////////////// 
//part related to ros imu publisher//? in rad or deg
imu_msg.orientation.z = yaw_rad* 180/M_PI;
imu_msg.header.stamp = n.now();
imu_pub.publish( &imu_msg );
/////////////////
//part related to xy publisher //wait
   pose_xy_msg.position.x=x;
   pose_xy_msg.position.y=y;
   pose_xy_msg.orientation.z=yaw_rad* 180/M_PI;
   pose.publish(&pose_xy_msg);
//////////////////
//
      n.spinOnce();


}
// y ISRs
void y_ISR_A(void){
 if(digitalRead(y_A)!= digitalRead(y_B))
  y_count++;
  else
  y_count--;
  }
void y_ISR_B(void){
  if(digitalRead(y_A) == digitalRead(y_B))
  y_count++;
  else
  y_count--;
  }



  
// x ISRs
void x_ISR_A(void){
 if(digitalRead(x_A)!= digitalRead(x_B))
  x_count++;
  else
  x_count--;
  }
void x_ISR_B(void){
  if(digitalRead(x_A) == digitalRead(x_B))
  x_count++;
  else
  y_count--;
  }
//////////////////////////////////////////////////////////////////////////////////////////
void get_xy(void) {
dy=(y_count-y_count_pre)*count_length;
dx=(x_count-x_count_pre)*count_length;
y=y_pre+dy*cos(yaw_rad)-dx*sin(yaw_rad);
x=x_pre+dy*sin(yaw_rad)+dx*cos(yaw_rad);
y_pre=y;
x_pre=x;
y_count_pre=y_count;
x_count_pre=x_count;
} 
  
