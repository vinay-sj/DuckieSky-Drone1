import rospy
from pidrone_pkg.msg import Mode
from geometry_msgs.msg import Pose, Twist

velocity = 1

def left():
    t = Twist()
    t.linear.x = -velocity
    t.linear.y = 0
    t.linear.z = 0
    t.angular.x = 0
    t.angular.y = 0
    t.angular.z = 0
    return t

def right():
    t = Twist()
    t.linear.x = velocity
    t.linear.y = 0
    t.linear.z = 0
    t.angular.x = 0
    t.angular.y = 0
    t.angular.z = 0
    return t

def forward():
    t = Twist()
    t.linear.x = 0
    t.linear.y = velocity
    t.linear.z = 0
    t.angular.x = 0
    t.angular.y = 0
    t.angular.z = 0
    return t

def back():
    t = Twist()
    t.linear.x = 0
    t.linear.y = -velocity
    t.linear.z = 0
    t.angular.x = 0
    t.angular.y = 0
    t.angular.z = 0
    return t

def landing():
    p = Pose()
    p.position.z = - 0.05
    p.position.x = 0
    p.position.y = 0
    p.orientation.x = 0
    p.orientation.y = 0
    p.orientation.z = 0
    p.orientation.w = 0
    return p

def arm():
    print "arm"
    mode = Mode()
    mode.mode = "ARMED"
    return mode

def takeoff():
    print "takeoff"
    mode = Mode()
    mode.mode = "FLYING"
    return mode

def disarming():
    print "disarm"
    mode = Mode()
    mode.mode = "DISARMED"
    return mode

class State:
    def __init__(self):
        self.run = True

    def infrared_callback(self, msg):
        if msg.range <= 0.15:
            self.run = False

def main():
    state = State()

    rospy.Subscriber('/pidrone/infrared', Range, state.infrared_callback)

    sq_pub = rospy.Publisher('/pidrone/desired/twist', Twist, queue_size=1)
    pose_pub = rospy.Publisher('/pidrone/desired/pose', Pose, queue_size=1)
    mode_pub = rospy.Publisher('/pidrone/desired/mode', Mode, queue_size=1)

    rate = rospy.Rate(60)

    l = left()
    r = right()
    f = forward()
    b = back()
    land = landing()
    disarm = disarming()

    # time in seconds for traveling each side of square
    time = 3 * 60

    # arm drone
    mode = arm()
    mode_pub.publish(mode)
    rate.sleep()

    # takeoff
    mode = takeoff()
    mode_pub.publish(mode)
    rate.sleep()

    start_time = rospy.get_time()

    while not rospy.is_shutdown():
        if state.run:
            duration = rospy.get_time() - start_time
            if duration < time:
                sq_pub.publish(f)
            elif duration >= time and duration < 2 * time:
                sq_pub.publish(r)
            elif duration >= 2 * time and duration < 3 * time:
                sq_pub.publish(b)
            elif duration >= 3 * time and duration < 4 * time:
                sq_pub.publish(l)
            else:
                pose_pub.publish(land)
            rate.sleep()
        else:
            # disable drone
            mode_pub.publish(disarm)
            break

if __name__ == '__main__':
    main()