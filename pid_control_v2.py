#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


rospy.init_node('pid_control')
bridge = CvBridge()

VELOCITY = 0.42
TURN_ANG_VEL = 1.5

location  = "start"
ready_to_turn = False
stopped = False
started = False

# prev_twist_x = np.zeros((1, 10))
# current_twist_ind = 0

position = np.array([0.0, 0.0]) # x, y
orientation = 0 # angle wrt x

positions = ["OR", "OT", "OL", "OB"] # Outer loop
current_pos_index = 0
turning = False
outer_radius = 1.22

next_turns = ["ORT", "OLT", "OLB", "ORB"]
keypoints = {
    "ORT": (outer_radius, -outer_radius),
    "ORB": (-outer_radius, -outer_radius),
    "OLT": (outer_radius, outer_radius),
    "OLB": (-outer_radius, outer_radius),
    "ORM": (-outer_radius/2, -outer_radius),
}
# Set position
def pos_callback(pos_msg):
    global position, orientation, location
    global current_pos, current_pos_index, keypoints

    current_pos = positions[current_pos_index]
    position[0], position[1] = pos_msg.pose.pose.position.x, pos_msg.pose.pose.position.y

    quaternion = ( pos_msg.pose.pose.orientation.x,
    pos_msg.pose.pose.orientation.y,
    pos_msg.pose.pose.orientation.z,
    pos_msg.pose.pose.orientation.w)
    orientation = euler_from_quaternion(quaternion)[2]
    if orientation < 0:
        orientation = orientation + 2*np.pi

    next_turn = keypoints[next_turns[current_pos_index]]

    if np.linalg.norm(position - next_turn) < 0.2:
        current_pos_index = (current_pos_index + 1)%4
        location = "corner"


def process_image(cv_image):
    cmd_pub.publish(get_final_twist(cv_image))


def get_final_twist(cv_image):
    global stopped, location, ready_to_turn
    frame_x = len(cv_image[0])

    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    lower_road_colour = np.array([0, 0, 79])
    upper_road_colour = np.array([0, 0, 96])

    mask = cv2.inRange(hsv,lower_road_colour,upper_road_colour)
    road_strip = mask

    if location in ["outside", "turning", "corner", "start"]:
        road_strip = road_strip[-450:, :]
        left_side = np.copy(road_strip[:, 0:frame_x/4])
        front_side = np.copy(road_strip[0:300, frame_x/3:-frame_x/3])
        road_strip[:, 0:frame_x/4] = 0
        
    elif location == "inside":
        road_strip = road_strip[-320:, :]
        road_strip[:, 0:frame_x/3] = 0
        road_strip[:, -frame_x/4:frame_x] = 0
        
    M = cv2.moments(road_strip)
    x_centroid = 0 if M["m00"] == 0 else int(M["m10"] / M["m00"])
    error = frame_x/2 - x_centroid

    twist = get_twist(error)

    # prev_twist_x[current_twist_ind] = twist.linear.x
    # current_twist_ind = (current_twist_ind + 1)%len(prev_twist_x)
    # twist.linear.x = np.mean(prev_twist_x)
    # is_pedestrian(cv_image)
    # cv2.putText(cv_image, str(is_pedestrian(cv_image)), (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
    #             0.6, (255, 255, 255), 2)
    # cv2.circle(road_strip, (x_centroid, 100), 10, (170,170,170), -1) 

    if location == "outside":
        cv2.putText(cv_image, str(np.count_nonzero(left_side) > 95000 and np.count_nonzero(front_side) > 51000), 
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    if location == "inside" or location == "turning":
        cv2.putText(cv_image, str(is_vehicle(hsv)), 
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(cv_image, str(location), 
        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    cv2.imshow("img", cv_image)
    cv2.waitKey(1)

    if location == "outside": 
        if ready_to_turn and np.count_nonzero(left_side) > 96500 and np.count_nonzero(front_side) > 55000:  # Turning point found
            print("Turning into inside")
            change_location("turning")

    # Deal with different locations
    if started:
        if location == "outside": # Stop if pedestrian is detected
            if is_pedestrian(hsv):
                stopped = True
            elif try_start(20):
                stopped = False

        elif location == "corner":
            twist = foward_and_left(0, 10, "outside")

        elif location == "start":
            twist = foward_and_left(10, 10, "outside")
            
        else:
            if location == "turning":
                twist = foward_and_left(0, 20, "inside")

            if is_vehicle(hsv):
                stopped = True
            elif try_start(20):
                stopped = False

        if stopped:
            twist = Twist()
        
        return twist
    return Twist()


# Update location
def change_location(new_loc):
    global location, VELOCITY
    if new_loc == "inside":
        VELOCITY = 0.21
    else:
        VELOCITY = 0.42
    location = new_loc

# Only start again if there are this many good frames in a row
start_count = 0
def try_start(max_count):
    global start_count
    start_count += 1
    if start_count >= max_count:
        start_count = 0
        return True
    return False

# Go forward, then left, then change location 
forward_loop_count = 0
turn_loop_count = 0
def foward_and_left(max_forward_count, max_turn_count, new_loc):
    global stopped, location, turn_loop_count, forward_loop_count
    if stopped:
        return Twist()

    twist = Twist()
    twist.linear.x = VELOCITY
    
    forward_loop_count += 1
    if forward_loop_count < max_forward_count:
        return twist

    turn_loop_count += 1
    twist.angular.z = TURN_ANG_VEL*3/2

    if turn_loop_count >= max_turn_count:
        change_location(new_loc)
        turn_loop_count = 0
        forward_loop_count = 0
        print("Finished turning")
    return twist


# Detect whether pedestrian is in middle of road
def is_pedestrian(hsv):
    frame_x = len(hsv[0])
    ped_fov_range = (frame_x/4, -frame_x/4)
    img_y_cutoff = -360, -200
    hsv = hsv[img_y_cutoff[0]:img_y_cutoff[1], ped_fov_range[0]:ped_fov_range[1]]

    # define range of blue color in HSV - pants of pedestrian
    lower_blue = np.array([100,0,46])
    upper_blue = np.array([106,255,60])

    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    if np.count_nonzero(mask_blue) < 180: # Not enough pixels for pedestrian to be detected
        return False

    # Check that pedestrian is in middle of road - between road lines
    M = cv2.moments(mask_blue)   
    x_centroid = 0 if M["m00"] == 0 else int(M["m10"] / M["m00"])
    y_centroid = 0 if M["m00"] == 0 else int(M["m01"] / M["m00"])

    upper_white = np.array([0,0,255])
    lower_white = np.array([0,0,252])
    road_lines = cv2.inRange(hsv, lower_white, upper_white)
    boundaries = np.nonzero(road_lines[y_centroid])[0] # Road line boundaries

    detected = False if len(boundaries) < 2 else boundaries[0] <= x_centroid <= boundaries[-1] # Check if pedestrian is between lines
    return detected

# Detect whether vehicle is directly in front of robot
def is_vehicle(hsv):
    # ped_fov_range = (325, 800)
    img_y_cutoff = -300, -100
    hsv = hsv[img_y_cutoff[0]:img_y_cutoff[1], :]
    # hsv = cv2.cvtColor(img_bottom, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV - pants of pedestrian
    lower_red = np.array([0,127,0])
    upper_red = np.array([5,255,255])

    lower_white = np.array([0, 0, 50])
    upper_white = np.array([255, 0, 67])

    mask = cv2.inRange(hsv, lower_red, upper_red) | cv2.inRange(hsv, lower_white, upper_white)

    return not np.count_nonzero(mask) < 3000 # Count pixels

def img_callback(image_msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
    except CvBridgeError as e:
       print(e)    
       return

    # cv2.imshow('image', cv_image)
    if not started:
        return
    process_image(cv_image)

# Generate twist from error
prev_error = 0
def get_twist(error):
    p = 0.01
    d = 0
    twist = Twist()
    twist.linear.x = VELOCITY
    twist.angular.z = error*p + d*(error - prev_error)
    return twist

# Start competition clock
def start_comp():
    global started

    license_pub.publish('Dante_Leo,password,0,START')
    started = True

    print("Start")
    print("First Turn")


sub_camera = rospy.Subscriber(
    "/R1/pi_camera/image_raw", Image, img_callback)
# sub_clock = rospy.Subscriber(
#     " /clock", Image, clock_callback)
cmd_pub = rospy.Publisher("/R1/cmd_vel", Twist, queue_size=10)
license_pub = rospy.Publisher("/license_plate", String, queue_size=5, latch=True)
sub_position = rospy.Subscriber(
    "/R1/odom", Odometry, pos_callback)

# while license_pub.get_num_connections() == 0:
#     print("Waiting for subscribers")
#     rospy.sleep(0.5)

start_comp()
start_time = rospy.get_rostime().secs
duration = 60*4 - 5 # seconds

r = rospy.Rate(0.5)
while not rospy.is_shutdown() and rospy.get_rostime().secs - start_time < duration:
    license_pub.publish('Dante_Leo,password,1,XR58')
    r.sleep()

# end_comp()

rospy.spin()
cv2.destroyAllWindows()
