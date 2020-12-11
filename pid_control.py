#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String

rospy.init_node('pid_control')
bridge = CvBridge()

# Navigation variables
GROUP_NAME = "Dante_Leo"
PASSWORD = "password"
TURN_ANG_VEL = 1.5

OUTSIDE_VELOCITY = 0.35
INSIDE_VELOCITY = 0.3
TURNING_VELOCITY = 0.42
STARTING_VELOCITY = 0.42

angular_vel = 1.5
velocity = 0.42
location  = "start"
ready_to_turn = False
stopped = False
started = False

# prev_twist_x = np.zeros((1, 10))
# current_twist_ind = 0

def process_image(cv_image):
    global stopped, location, ready_to_turn
    frame_x = len(cv_image[0])

    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    lower_road_colour = np.array([0, 0, 79])
    upper_road_colour = np.array([0, 0, 96])

    mask = cv2.inRange(hsv,lower_road_colour,upper_road_colour)
    road_strip = mask

    # Cut road
    if location in ["outside", "turning", "start"]:
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

    if location == "outside":
        cv2.putText(cv_image, str(np.count_nonzero(left_side) > 95000 and np.count_nonzero(front_side) > 51000), 
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    if location == "inside" or location == "turning":
        cv2.putText(cv_image, str(is_vehicle(hsv)), 
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(cv_image, str(location), 
        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(cv_image, "Ready to Turn: " + str(ready_to_turn), 
        (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
    cv2.imshow("img", cv_image)
    cv2.waitKey(1)

    # Locate turning point if needed
    if ready_to_turn and location == "outside" and np.count_nonzero(left_side) > 95000 and np.count_nonzero(front_side) < 51000:  # Turning point found
        print("Turning into inside")
        cmd_pub.publish(Twist())
        change_location("turning")

    # Deal with different locations
    if started:
        if location == "outside": # Stop if pedestrian is detected
            if is_pedestrian(hsv):
                stopped = True
            elif stopped and try_start(20):
                stopped = False

        elif location == "start":
            twist = foward_and_left(6, 10, "outside")
            
        elif location == "spin":
            twist = sweep(30,"inside")

        else:
            if location == "turning":
                twist = foward_and_left(0, 20, "inside")

            if is_vehicle(hsv):
                stopped = True
                change_location("spin")
            
            elif stopped and try_start(70):
                stopped = False

        if stopped and not location == "spin":
            twist = Twist()
        
        cmd_pub.publish(twist)

# Update location
def change_location(new_loc):
    global location, velocity, p
    if new_loc == "inside":
        print("Inside")
        velocity = INSIDE_VELOCITY
    elif new_loc == "turning":
        velocity = TURNING_VELOCITY
    else:
        velocity = OUTSIDE_VELOCITY
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
    twist.linear.x = velocity
    
    forward_loop_count += 1
    if forward_loop_count < max_forward_count:
        return twist

    turn_loop_count += 1
    twist.angular.z = angular_vel*3/2

    if turn_loop_count >= max_turn_count:
        change_location(new_loc)
        turn_loop_count = 0
        forward_loop_count = 0
        print("Finished turning")
    return twist

# Sweep back and forth
spin_count = 0
spin_opposite = False
def sweep(time, cur_loc):
    global spin_count, spin_opposite, stopped
    spin_count +=1
    
    twist = Twist()
    twist.angular.z = angular_vel if spin_opposite else -angular_vel

    if not spin_opposite and spin_count >= time:
        spin_count = 0
        spin_opposite = True
    elif spin_count >= time:
        spin_count = 0
        change_location(cur_loc)
        spin_opposite = False
        stopped = False

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
    blue_count = np.count_nonzero(mask_blue)
    if blue_count < 180: # Not enough pixels for pedestrian to be detected
        return False

    # print(np.count_nonzero(mask_blue))

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
    if not started: # Do nothing if not started
        return

    process_image(cv_image)

# Generate twist from error
prev_error = 0
p = 0.01
def get_twist(error):
    d = 0
    twist = Twist()
    twist.linear.x = velocity
    # print(error)
    twist.angular.z = error*p + d*(error - prev_error)
    return twist

def frame_command_callback(str_msg):
    global ready_to_turn
    if str_msg.data == "stop":
        end_comp()
    elif str_msg.data == "turn":
        go_inner()
    return None

# Start competition clock
def start_comp():
    global started
    license_pub.publish(",".join([GROUP_NAME, PASSWORD, "0", "START"]))
    started = True

    print("Start")
    print("First Turn")

# Stop competition clock
def end_comp():
    global started
    started= False
    print("Ending")
    cmd_pub.publish(Twist()) # Stop
    license_pub.publish(",".join([GROUP_NAME, PASSWORD, "0", "END"]))

# Go towards inner loop
def go_inner():
    global ready_to_turn
    ready_to_turn = True


sub_camera = rospy.Subscriber(
    "/R1/pi_camera/image_raw", Image, img_callback)
sub_frame_command = rospy.Subscriber("/plate_reader_command", String, frame_command_callback)
cmd_pub = rospy.Publisher("/R1/cmd_vel", Twist, queue_size=10)
license_pub = rospy.Publisher("/license_plate", String, queue_size=5, latch=True)

while license_pub.get_num_connections() == 0:
    print("Waiting for subscribers")
    rospy.sleep(0.5)

start_comp()

# r = rospy.Rate(0.5)
# while not rospy.is_shutdown() and rospy.get_rostime().secs - start_time < duration:
#     r.sleep()
# end_comp()

rospy.spin()

cv2.destroyAllWindows()
