#include "ros.h"
#include <geometry_msgs/Twist.h>

#include <ax12.h>

//init position
ros::NodeHandle nh;
int a = 400 , b = 200;
int c = 400 , d = 519;
int g = 684 ;
void callback(geometry_msgs::Twist &msg){
  a = msg.linear.x;
  b = msg.linear.y;
  c = msg.linear.y - 30;
  g = msg.angular.x;
}


ros::Subscriber<geometry_msgs::Twist> sub("detecXY", callback);


const int servo = 5;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  nh.initNode();
  nh.subscribe(sub);
  dxlSetGoalSpeed(DXL_BROADCAST,50);




}



unsigned long LastMotionTime = 0;
const unsigned long millisecondsBetweenMotions = 1000;

void loop() {
  int run[5][2] = {{2,a},{3,b},{6,c},{7,d},{8,g}};

  unsigned long currentTime = millis();
  if(currentTime - LastMotionTime < millisecondsBetweenMotions){
    return;
  }



  if(dxlGetMoving(2)){
    return;
  }
  if(b >= 500 ){
    dxlSetTorqueEnable(DXL_BROADCAST,false);
  }else{
    dxlSetTorqueEnable(DXL_BROADCAST,true);
    dxlSyncWritePosition(run, servo);

  }
  nh.spinOnce();


  // delay(500);


  // Serial.println(a);
  

}




