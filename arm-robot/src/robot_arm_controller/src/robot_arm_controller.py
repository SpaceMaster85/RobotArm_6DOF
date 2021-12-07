#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
import serial
import time
import lewansoul_lx16a
import enum
import actionlib


from control_msgs.msg    import FollowJointTrajectoryGoal
from control_msgs.msg    import FollowJointTrajectoryResult
from control_msgs.msg    import FollowJointTrajectoryFeedback
from control_msgs.msg    import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory


def counts2rad(counts):
    return -counts*0.0041887902048


def rad2counts(rads):
    return rads*238.732414637067382

def clamp(range_min, range_max, value):
    return min(range_max, max(range_min, value))     

CW = 1
CCW = -1


# base_link2rotary_plate 1
# rotary_plate2upper_arm 2 21
# upper_arm2elbow 3 31
# elbow2forearm 4
# forearm2wrist 5
# wrist2hand 6
# rad 135==2.3561944901923
# rad 1 == 424,413181578395654


class Joints:

    def __init__(self, controller):
        self.controller = controller
        self.joint_list = []

    def clear(self):
        self.joint_list = []

    def append(self, joint):
        self.joint_list.append(joint)

    def get_joint_list(self):
        return self.joint_list

    def get_name_list(self):
        name_list = []
        for joint in self.joint_list:
            name_list.append(joint.name)
        return name_list

    def get_position_list(self):
        return [joint.get_joint_position() for joint in self.joint_list]
        # position_list = []
        # for joint in self.joint_list:
        #     position_list.append(joint.get_joint_position())
        # # print(position_list)
        # return position_list

    def move_start(self):
        self.controller.move_start()

    def move_prepare(self, positions, time=-1):
        for i, joint in enumerate(self.joint_list):
            joint.set_joint_position_prepare(-positions[i], time=time)

    def move(self, positions, time=-1):
        for i, joint in enumerate(self.joint_list):
            joint.set_joint_position(-positions[i], time=time)

    def motor_on(self):
        for joint in self.joint_list:
            joint.motor_on()

    def motor_off(self):
        # map(lambda j: j.motor_off(), self.joint_list)
        for joint in self.joint_list:
            joint.motor_off()

    def calibrate(self):
        for joint in self.joint_list:
            joint.calibrate()
            # joint.set_joint_position(0)
        # self.controller.move_start()
        self.motor_off()

    def reset_offset(self):
        for joint in self.joint_list:
            joint.set_position_offset(0)
        self.motor_off()


class Joint:

    def __init__(self, controller, name, motors):
        self.name = name
        self.motors = []
        for conf in motors:
            self.motors.append(Motor(controller, conf[0], conf[1]))

    def get_joint_position(self):
        return counts2rad(self.motors[0].get_position())

    def set_joint_position_prepare(self, position, time=-1):
        for motor in self.motors:
            motor.move_prepare(rad2counts(position), time=time)

    def set_joint_position(self, position, time=-1):
        for motor in self.motors:
            motor.move(rad2counts(position), time=time)

    def set_position_offset(self, position):
        for motor in self.motors:
            motor.set_position_offset(position)

    def calibrate(self):
        for motor in self.motors:
            motor.calibrate()

    def motor_on(self):
        for motor in self.motors:
            motor.motor_on()

    def motor_off(self):
        for motor in self.motors:
            motor.motor_off()


class Motor:
    def __init__(self, controller, id, direction):
        self.id = id
        self.dir = direction
        self.controller = controller
        self.proxy = controller.servo(id)

    def get_position(self):
        try:
            position = (self.proxy.get_position()-500)*self.dir
        except Exception as e:
            print("Error Motor ID:", self.id)
            return 0
        else:
            return position

    def move_prepare(self, position, time):
        try:
            if time == -1:
                time = int(abs(position-self.get_position())*5)
            self.proxy.move_prepare(int(self.dir*position+500), time=time)
        except Exception as e:
            print("Error Motor ID:", self.id)

    def move(self, position, time):
        try:
            if time == -1:
                time = int(abs(position-self.get_position())*5)
            self.proxy.move(int(self.dir*position+500), time=time)
        except Exception as e:
            print("Error Motor ID:", self.id)


    def motor_on(self):
        try:
            self.proxy.motor_on()
        except Exception as e:
            print("Error Motor ID:", self.id)

    def motor_off(self):
        try:
            self.proxy.motor_off()
        except Exception as e:
            print("Error Motor ID:", self.id)

    def set_position_offset(self, position):
        try:
            self.proxy.set_position_offset(position)
            self.proxy.save_position_offset()
        except Exception as e:
            print("Error Motor ID:", self.id)

    def calibrate(self):
        try:
            offset = self.proxy.get_position_offset()
            self.set_position_offset(self.get_position()*self.dir+offset)
            self.proxy.save_position_offset()
        except Exception as e:
            print("Error Motor ID:", self.id)


class Robot_arm_controller:
    def __init__(self):
        rospy.init_node('robot_arm_controller', anonymous=True)

        rospy.Service('calibrate', Trigger, self.calibrate_cb)
        rospy.Service('stop', Trigger, self.stop_cb)
        rospy.Subscriber("desired_joint_states", JointState, self.drive_cb)

        self.pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        
        self._as = actionlib.SimpleActionServer('robot_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()


        self.positions = []
        controller = lewansoul_lx16a.ServoController(
            serial.Serial(rospy.get_param('~port', '/dev/ttyUSB0'), rospy.get_param('~baud', 115200), timeout=rospy.get_param('~timeout', 1)), timeout=1
        )

        self.joints = Joints(controller)
        self.joints.append(
            Joint(controller, "base_link2rotary_plate", [(1, CW)]))
        self.joints.append(
            Joint(controller, "rotary_plate2upper_arm", [(2, CW), (21, CCW)]))
        self.joints.append(
            Joint(controller, "upper_arm2elbow", [(3, CCW), (31, CW)]))
        self.joints.append(Joint(controller, "elbow2forearm", [(4, CW)]))
        self.joints.append(Joint(controller, "forearm2wrist", [(5, CCW)]))
        self.joints.append(Joint(controller, "wrist2hand", [(6, CW)]))
        if rospy.get_param('~gripper', False):
            self.joints.append(Joint(controller, "gripper", [(7, CW)]))            
       

        # self.joints.move([0,0,0,0,0,0], 500)
        # time.sleep(0.1)
        # self.joints.move([0.2,0.2,0.2,0.2,0.2,0.2], 500)
        # time.sleep(0.1)        
        # self.joints.move([0.4,0.4,0.4,0.4,0.4,0.4], 500)
        # time.sleep(0.1)
        # self.joints.move([0.6,0.6,0.6,0.6,0.6,0.6], 500)
        # time.sleep(0.1)
        # self.joints.move([0.8,0.8,0.8,0.8,0.8,0.8], 500)
        # time.sleep(0.1)
        # self.joints.move([0.6,0.6,0.6,0.6,0.6,0.6], 500)
        # time.sleep(0.1)
        # self.joints.move([0.8,0.8,0.8,0.8,0.8,0.8], 500)


        # self.joints.reset_offset()

        self.joints.motor_off()

    def calibrate_cb(self, request):
        self.publish = False
        self.joints.calibrate()
        self.publish = True
        return TriggerResponse(
            success=True,
            message="Robot arm was calibrated"
        )

    def stop_cb(self, request):
        self.publish = False
        self.joints.motor_off()
        self.publish = True
        return TriggerResponse(
            success=True,
            message="Robot arm stopped"
        )

    def drive_cb(self, data):

        if (self.positions != data.position):
            self.positions = data.position
            self.joints.move_prepare(data.position)
            self.joints.move_start()

 

    def rearrange(self, joint_trajectory):

        mapping = [joint_trajectory.joint_names.index(j) for j in self.joints.get_name_list()]

        for point in joint_trajectory.points:
            temp_positions = []
            temp_velocities = []
            temp_accelerations = []
            temp_effort = []
            for i in range(len(point.positions)):
                temp_positions.append(point.positions[mapping[i]])
            for i in range(len(point.velocities)):
                temp_velocities.append(point.velocities[mapping[i]])
            for i in range(len(point.accelerations)):
                temp_accelerations.append(point.accelerations[mapping[i]])
            for i in range(len(point.effort)):
                temp_effort.append(point.effort[mapping[i]])
            point.positions = temp_positions
            point.velocities = temp_velocities
            point.accelerations = temp_accelerations
            point.effort = temp_effort
        joint_trajectory.joint_names = self.joints.get_name_list()
        
    def execute_cb(self, goal):
        self._feedback = FollowJointTrajectoryFeedback()
        self._result = FollowJointTrajectoryResult()
        # It is required to rearrange the arrays because MoveIt doesn't guarantee orden preservation
        self.rearrange(goal.trajectory)      
        # A trajectory needs at least 2 points		
        if len(goal.trajectory.points) < 2:
            return
        #print(goal.trajectory.points)
        time_start = rospy.Time.from_sec(time.time())     
        last_point = goal.trajectory.points[0]
        last_time = rospy.get_time()
        

        time_benchmark = []

        for point in goal.trajectory.points[1:]:     

            time_to_wait = point.time_from_start.to_sec() - last_point.time_from_start.to_sec() 
            #print(time_to_wait)
            time_stamp = rospy.get_time() 
            #time_1 = rospy.get_time() 
            self.joints.move(point.positions, int(time_to_wait *1000))
            #time_benchmark.append(rospy.get_time()-time_1)
            #print( (time_to_wait + time_stamp) - rospy.get_time())
            timing = clamp(0.0,1000.0, ((time_to_wait + time_stamp) - rospy.get_time()-0.015))
            time.sleep(timing)
            #self.joints.move_start()   
            #print(timing)         
            #print((point.time_from_start.to_sec() - last_point.time_from_start.to_sec()) *1000)
            #print(point.time_from_start - last_point.time_from_start)
            #rospy.sleep(point.time_from_start - last_point.time_from_start )       

   
            last_point = point
        #print(time_benchmark)
        point = goal.trajectory.points[-1]

        self._feedback.joint_names = self.joints.get_name_list()
        self._feedback.desired = point
        self._feedback.actual.positions = self.joints.get_position_list()          
        self._feedback.actual.velocities = []
        self._feedback.actual.time_from_start = rospy.Time.from_sec(time.time()) - time_start
        self._as.publish_feedback(self._feedback)        
        #    last_point = point      

        # ------------- Wait until the termination while providing feedback      
        #last_point = goal.trajectory.points[0]
        #for point in goal.trajectory.points[1:]:

        	# ---------------------------------------
            #last_point = point       
        # ---------------------------------------
        # Result
        self._result.error_code = 0
        self._as.set_succeeded(self._result)
        # ---------------------------------------

    def run(self):


        rate = rospy.Rate(10)  # 10hz

        self.joints.motor_on()

        msg = JointState()
        msg.name = self.joints.get_name_list()
        msg.velocity = []
        msg.effort = []

        while not rospy.is_shutdown():
            msg.position = self.joints.get_position_list()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'base_link'
            self.pub.publish(msg)

            rate.sleep()


if __name__ == '__main__':
    try:
        control = Robot_arm_controller()
        control.run()
    except rospy.ROSInterruptException:
        pass
