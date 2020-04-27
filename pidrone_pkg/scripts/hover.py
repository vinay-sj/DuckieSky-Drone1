import sys
import signal
import rospy
from pidrone_pkg.msg import Mode
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Empty

class Hover:
    def __init__(self):
        self.run = True
        self.takeoff_happened = False
        self.is_landing = False
        self.last_height = 0.1
        self.land_drone = self.landing()
        self.arm_drone = self.arm()
        self.takeoff_drone = self.takeoff()
        self.disarm_drone = self.disarming()
        self.twist = self.get_twist()
        self.pos = self.get_pos()
        self.pose_pub = rospy.Publisher('/pidrone/desired/pose', Pose, queue_size=1)
        self.mode_pub = rospy.Publisher('/pidrone/desired/mode', Mode, queue_size=1)
        self.velocity_pub = rospy.Publisher('/pidrone/desired/twist', Twist, queue_size=1)
        self.positionControlPub = rospy.Publisher('/pidrone/position_control', Bool, queue_size=1)
        self.resetPositionPub = rospy.Publisher('/pidrone/reset_transform', Empty, queue_size=1)
        self.positionControl = Bool()
        self.positionControl.data = False

    def toggleControl(self):

        if self.positionControl.data is False:
            self.resetPositionPub.publish(Empty())
            self.positionControl.data = True
        else:
            self.positionControl.data = False
        self.positionControlPub.publish(self.positionControl)

    def infrared_callback(self, msg):
        if self.is_landing:
            if self.last_height - msg.range >= 0.04:
                print "publishing next drop command"
                self.pose_pub(self.land_drone)
                self.last_height = msg.range
        else:
            self.last_height = msg.range
        if msg.range <= 0.15:
            if self.takeoff_happened:
                print "setting run to false, range is ", msg.range
                self.run = False
        if msg.range > 0.2:
            self.takeoff_happened = True

    def ctrl_c_handler(self, signal, frame):
        print "Caught ctrl-c stopping node"
        self.mode_pub.publish(self.disarm_drone)
        sys.exit()

    def get_twist(self):
        twistMsg = Twist()
        twistMsg.linear.x = 0
        twistMsg.linear.y = 0
        twistMsg.linear.z = 0
        twistMsg.angular.x=0
        twistMsg.angular.y=0
        twistMsg.angular.z=0
        return twistMsg

    def get_pos(self):
        poseMsg = Pose()
        poseMsg.position.x=0
        poseMsg.position.y=0
        poseMsg.position.z=0
        poseMsg.orientation.x=0
        poseMsg.orientation.y=0
        poseMsg.orientation.z=0
        poseMsg.orientation.w=0
        return poseMsg

    def landing(self):
        p = Pose()
        p.position.z = - 0.05
        p.position.x = 0
        p.position.y = 0
        p.orientation.x = 0
        p.orientation.y = 0
        p.orientation.z = 0
        p.orientation.w = 0
        return p

    def arm(self):
        mode = Mode()
        mode.mode = "ARMED"
        return mode

    def takeoff(self):
        mode = Mode()
        mode.mode = "FLYING"
        return mode

    def disarming(self):
        mode = Mode()
        mode.mode = "DISARMED"
        return mode

    def mode_callback(self, msg):
        pass
        #print "current mode: ", msg.mode

    def publishDisarm(self):
        if self.positionControl.data:
            self.pose_pub.publish(self.pos)
        else:

            self.velocity_pub.publish(self.twist)
        self.mode_pub.publish(self.disarm_drone)

    def publishArm(self):
        if self.positionControl.data:
            self.pose_pub.publish(self.pos)
        else:

            self.velocity_pub.publish(self.twist)
        self.mode_pub.publish(self.arm_drone)

    def publishTakeOff(self):
        if self.positionControl.data:
            self.pose_pub.publish(self.pos)
        else:

            self.velocity_pub.publish(self.twist)
        self.mode_pub.publish(self.takeoff_drone)

    def publishLanding(self):
        self.velocity_pub.publish(self.land_drone)

def main():
    rospy.init_node("hover")
    hover = Hover()

    rospy.Subscriber('/pidrone/infrared', Range, hover.infrared_callback)
    rospy.Subscriber('/pidrone/mode', Mode, hover.mode_callback)

    signal.signal(signal.SIGINT, hover.ctrl_c_handler)

    rate = rospy.Rate(60)

    # time to hover
    time = 4

    rospy.sleep(2)

    start_time = rospy.get_time()
    last_mode = "DISARMED"
    hover.toggleControl()
    while not rospy.is_shutdown():
        if hover.run:
            duration = rospy.get_time() - start_time
            if duration < time:
                desired_mode = "ARMED"
                if desired_mode != last_mode:
                    print "sending arm command"
                    print "arm: ", hover.arm_drone.mode
                    hover.publishArm()
                    print "arm command sent"
                    last_mode = desired_mode
            elif duration >= time and duration < 2 * time:
                desired_mode = "FLYING"
                if desired_mode != last_mode:
                    print "sending takeoff command"
                    hover.publishTakeOff()
                    last_mode = desired_mode
            elif duration >= 2 * time:
                print "descending"
                # if first time in this part of the loop publish down command
                if not hover.is_landing:
                    hover.toggleControl()
                    hover.publishLanding()
                    hover.is_landing = True

        else:
            # disable drone
            desired_mode = "DISARMED"
            if desired_mode != last_mode:
                print "sending disarm command"
                hover.publishDisarm()
                last_mode = desired_mode
                break

        rate.sleep()

if __name__ == '__main__':
    main()
