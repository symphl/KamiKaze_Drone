#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from vision_msgs.msg import Detection2DArray
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
import math
import time
import sys
import termios
import tty
import select


class KamikazeDronePosition:
    def __init__(self):
        rospy.init_node('kamikaze_drone_position_node')

        # Switch to position control like the working example
        self.pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        rospy.Subscriber('/yolo/truck_bbox', Detection2DArray, self.truck_callback)
        rospy.Subscriber('/mavros/state', State, self.state_cb)

        self.arming_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        self.current_state = State()

        self.target_detected = False
        self.image_width = 640
        self.image_height = 480

        self.target_pose = PoseStamped()  # Target position
        self.smoothing_factor = 0.15  # Lower = smoother movement but less responsive
        # Position setpoint (similar to working example)
        self.pose = PoseStamped()
        self.pose.header.frame_id = "map"
        self.pose.pose.position.x = 0.0
        self.pose.pose.position.y = 0.0
        self.pose.pose.position.z = 10.0  # Start at 2 meters height
        
        # Initialize orientation quaternion (forward-facing)
        self.pose.pose.orientation.w = 1.0
        self.pose.pose.orientation.x = 0.0
        self.pose.pose.orientation.y = 0.0
        self.pose.pose.orientation.z = 0.0
        
        self.yaw_angle = 0.0  # Current yaw in radians
        self.pitch_angle = 0.0
        # Control parameters
        self.position_step = 1.20  # Position change per detection
        self.scanning_step = 0.04  # Yaw change per scan step (radians)
        self.climb_duration = 5.0
        self.start_time = None
        
        self.last_known_direction = None  # Store last known direction vector
        self.target_lost_time = 0  # Time since target was lost
        self.persistence_duration = 1.75  # Continue moving in last direction for 3 seconds
        # PID controller variables
        self.integral_error_x = 0.0
        self.integral_error_y = 0.0
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.integral_limit = 100.0  # Adjust as needed
        
        # PID gains for yaw control
        self.Kp_yaw = 0.0030  # Proportional gain
        self.Ki_yaw = 0.00015  # Integral gain
        self.Kd_yaw = 0.007  # Derivative gain
        self.last_detection_time = rospy.Time.now()
        self.rate = rospy.Rate(20)  # 20 Hz

        # Wait for FCU connection first
        self.wait_for_connection()
        
        # Initialize start time after connection
        self.start_time = rospy.Time.now()
        
        # Start publishing setpoints continuously at 20Hz
        self.timer = rospy.Timer(rospy.Duration(0.05), self.publish_setpoint)
        
        rospy.loginfo("⏳ Publishing setpoints for 5 seconds before attempting OFFBOARD...")
        rospy.sleep(5.0)  # Send setpoints for 5 seconds (longer than the 2s before)
        
        # Try to set offboard mode and arm
        self.set_offboard_and_arm()

    def state_cb(self, msg):
        prev_state = self.current_state
        self.current_state = msg
        
        # Report mode and armed status changes
        if prev_state.mode != self.current_state.mode:
            rospy.loginfo(f"Flight mode changed from {prev_state.mode} to {self.current_state.mode}")
            
        if prev_state.armed != self.current_state.armed:
            arming_status = "ARMED" if self.current_state.armed else "DISARMED"
            rospy.loginfo(f"Arming status changed to: {arming_status}")
            
        if self.start_time is None:
            return

        # Keep trying to arm and set offboard if not already
        if not self.current_state.armed or self.current_state.mode != "OFFBOARD":
            if rospy.Time.now() - self.start_time > rospy.Duration(10.0):  # Wait 10s before retrying
                self.set_offboard_and_arm()

    def wait_for_connection(self):
        rospy.loginfo("📡 Waiting for FCU connection...")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.current_state.connected:
            rate.sleep()
        rospy.loginfo("✅ FCU connected!")

    def set_offboard_and_arm(self):
        if self.current_state.mode != "OFFBOARD":
            # Try switching to OFFBOARD mode
            rospy.loginfo("🔁 Setting mode: OFFBOARD")
            try:
                offb_result = self.set_mode_srv(custom_mode="OFFBOARD")
                rospy.loginfo(f"Mode set result: {offb_result}")
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
                return
                
        # Mode should be OFFBOARD now, try arming
        if not self.current_state.armed:
            rospy.loginfo("🟢 Attempting to arm...")
            try:
                arm_result = self.arming_srv(True)
                rospy.loginfo(f"Arming result: {arm_result}")
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")

    def truck_callback(self, msg):
        found_truck = False

        for detection in msg.detections:
            for hypothesis in detection.results:
                if hypothesis.id == 7:  # Truck ID in COCO
                    found_truck = True
                    self.last_detection_time = rospy.Time.now()

                    bbox = detection.bbox
                    error_x = bbox.center.x - (self.image_width / 2)
                    error_y = bbox.center.y - (self.image_height / 2)

                    # --- YAW adjustment (left/right) ---
                    if abs(error_x) > 30:
                        self.integral_error_x += error_x
                        self.integral_error_x = max(min(self.integral_error_x, self.integral_limit), -self.integral_limit)
                        derivative_x = error_x - self.prev_error_x

                        yaw_adjustment = (self.Kp_yaw * error_x +
                                        self.Ki_yaw * self.integral_error_x +
                                        self.Kd_yaw * derivative_x)
                        yaw_adjustment = max(min(yaw_adjustment, 0.05), -0.05)

                        # Fix the yaw direction: negative adjustment for left turn when error is negative
                        self.yaw_angle -= yaw_adjustment
                        self.prev_error_x = error_x

                    # --- VERTICAL adjustment (up/down) ---
                    if abs(error_y) > 30:
                        # Calculate vertical adjustment
                        # Positive error_y means target is below center, so drone should descend
                        # Negative error_y means target is above center, so drone should ascend
                        vertical_speed = 0.2  # Base vertical movement speed (m/s)
                        
                        # Scale speed based on how far off-center the target is
                        vertical_adjustment = vertical_speed * (error_y / (self.image_height / 2))
                        
                        # Limit maximum vertical adjustment to prevent rapid changes
                        vertical_adjustment = max(min(vertical_adjustment, 0.5), -0.5)
                        
                        # Apply vertical adjustment directly to position
                        self.pose.pose.position.z -= 0.75*vertical_adjustment  # Subtract to descend when error_y is positive
                        
                        rospy.loginfo(f"Vertical adjustment: {vertical_adjustment:.2f} m - Height: {self.pose.pose.position.z:.2f} m")
                        self.prev_error_y = error_y
                    
                    # Set orientation based on yaw angle
                    cy = math.cos(self.yaw_angle * 0.5)
                    sy = math.sin(self.yaw_angle * 0.5)
                    
                    # Set quaternion for orientation (only yaw, no pitch adjustment in orientation)
                    self.pose.pose.orientation.w = cy
                    self.pose.pose.orientation.x = 0.0
                    self.pose.pose.orientation.y = 0.0
                    self.pose.pose.orientation.z = sy

                    # Calculate forward movement direction based on yaw
                    self.last_known_direction = {
                        'x': math.cos(self.yaw_angle),
                        'y': math.sin(self.yaw_angle),
                        'z': 0.0  # Vertical movement handled separately now
                    }

                    # Move if target is mostly centered horizontally
                    if abs(error_x) < 100:
                        # Adjust forward speed based on horizontal alignment
                        forward_speed = self.position_step * (1.0 - abs(error_x)/200.0)
                        forward_speed = max(forward_speed, 0.25)

                        self.pose.pose.position.x += forward_speed * self.last_known_direction['x']
                        self.pose.pose.position.y += forward_speed * self.last_known_direction['y']
                        rospy.loginfo(f"🎯 Moving toward target at speed {forward_speed:.2f}")
                    else:
                        rospy.loginfo("🎯 Target seen but not centered – adjusting orientation")

                    break
            if found_truck:
                break

        self.target_detected = found_truck
        if not found_truck:
            self.integral_error_x = 0.0
            self.integral_error_y = 0.0
            self.target_lost_time = (rospy.Time.now() - self.last_detection_time).to_sec()
            rospy.loginfo_throttle(1, f"No truck detected. Time since last seen: {self.target_lost_time:.2f}s")


    # Then update your publish_setpoint method:
    def publish_setpoint(self, event):
        if self.start_time is None:
            return
            
        time_elapsed = (rospy.Time.now() - self.start_time).to_sec()
        time_since_last_seen = (rospy.Time.now() - self.last_detection_time).to_sec()

        # Always update timestamp
        self.pose.header.stamp = rospy.Time.now()
        
        if time_elapsed < self.climb_duration:
            # Initial takeoff phase - just maintain height
            rospy.loginfo_throttle(1, f'🚁 Taking off to {self.pose.pose.position.z} meters...')
        elif self.target_detected:
            # Target tracking phase - updates handled in truck_callback
            pass
        elif time_since_last_seen < self.persistence_duration and self.last_known_direction:
            # Continue in last known direction for persistence_duration seconds
            # Use reduced speed to avoid overshooting
            forward_speed = 0.1
            self.pose.pose.position.x += forward_speed * self.last_known_direction['x']
            self.pose.pose.position.y += forward_speed * self.last_known_direction['y']
            rospy.loginfo_throttle(1, "🔄 Target lost but continuing in last known direction...")
        else:
            # Search phase - rotate to scan for targets
            # Use a slower scanning speed if we recently saw the target
            if time_since_last_seen < 5.0:  # Recently saw target
                scan_speed = self.scanning_step * 0.5  # Slower scan when recently seen
            else:
                scan_speed = self.scanning_step
                
            self.yaw_angle += scan_speed
            if self.yaw_angle > 2 * math.pi:
                self.yaw_angle -= 2 * math.pi
                
            # Update orientation quaternion
            self.pose.pose.orientation.z = math.sin(self.yaw_angle / 2.0)
            self.pose.pose.orientation.w = math.cos(self.yaw_angle / 2.0)
            
            rospy.loginfo_throttle(1, "🔍 Scanning for target...")
        
        # Publish the position setpoint
        self.pos_pub.publish(self.pose)

    def run(self):
        rospy.loginfo("📢 Kamikaze Drone running! Press Ctrl+C to stop")
        rospy.spin()

if __name__ == '__main__':
    try:
        drone = KamikazeDronePosition()
        drone.run()
    except rospy.ROSInterruptException:
        pass
