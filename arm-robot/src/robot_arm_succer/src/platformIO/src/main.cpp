#include <Arduino.h>
/*
 * rosserial Service Server
 */

#include <ros.h>
#include <control_msgs/GripperCommand.h>

#define VALVE 2
#define PUMP 3

ros::NodeHandle  nh;
//using rosserial_arduino::Test;


void gripperCommand_cb(const control_msgs::GripperCommand& succer_msg){
  if (succer_msg.position == 0)
  {
      digitalWrite(PUMP, LOW);
      digitalWrite(VALVE, HIGH);
      delay(1000);
      digitalWrite(VALVE, LOW);
  }
  else
  {
      digitalWrite(PUMP, HIGH);
      digitalWrite(VALVE, LOW);
  }

}

ros::Subscriber<control_msgs::GripperCommand> sub_succer("grippercommand", gripperCommand_cb );




void setup()
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub_succer);
  pinMode(VALVE, OUTPUT);
  pinMode(PUMP, OUTPUT);
}

void loop()
{
  nh.spinOnce();
  delay(10);
}

