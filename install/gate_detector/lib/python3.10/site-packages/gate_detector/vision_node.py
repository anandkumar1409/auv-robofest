import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped
from orca_msgs.action import TargetMode
import cv2
import numpy as np

class SmartGateNode(Node):
    def __init__(self):
        super().__init__('smart_gate_node')
        
        # --- TUNING ---
        self.TARGET_DEPTH = -1.0  # Meters
        self.DEPTH_TOLERANCE = 0.1
        self.ALIGN_TOLERANCE = 40 # Pixels (Dead zone width)
        
        # --- QOS PROFILE (THE FIX) ---
        # This allows the node to accept "Best Effort" data from MAVROS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # --- ROS ---
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # Use the new qos_profile here!
        self.pose_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, qos_profile)
        
        self.current_z = 0.0
        self.sensor_received = False 

        # --- MODE SWITCH ---
        self._action_client = ActionClient(self, TargetMode, '/set_target_mode')
        self.set_auv_mode()

        # --- CAMERA ---
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open Webcam!")

        # --- LOOP ---
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Smart Gate Node: QoS Fixed + Target Box Added")

    def set_auv_mode(self):
        self.get_logger().info('Waiting for Orca Manager...')
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("Mode Switcher timed out. Ensure Orca is running!")
            return
        
        goal_msg = TargetMode.Goal()
        goal_msg.target_mode = TargetMode.Goal.ORCA_MODE_AUV
        self._action_client.send_goal_async(goal_msg)

    def pose_callback(self, msg):
        self.current_z = msg.pose.position.z
        self.sensor_received = True

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret: return

        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape
        center_x = w // 2
        twist = Twist()

        # --- 1. DEPTH CONTROL ---
        if self.sensor_received:
            if self.current_z > (self.TARGET_DEPTH + self.DEPTH_TOLERANCE): 
                twist.linear.z = -0.3 # Dive
                depth_status = "DIVING"
            elif self.current_z < (self.TARGET_DEPTH - self.DEPTH_TOLERANCE):
                twist.linear.z = 0.3 # Pull Up
                depth_status = "CORRECTING UP"
            else:
                twist.linear.z = 0.0
                depth_status = "LOCKED"
        else:
            # Blind dive (be careful!) or just hover 0
            # Since you said you are happy with depth, we will allow small dive
            twist.linear.z = -0.1 
            depth_status = "NO SENSOR (Blind)"

        # --- 2. VISION LOGIC ---
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([5, 150, 150]), np.array([25, 255, 255]))
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        valid_objs = []
        for cnt in contours:
            if cv2.contourArea(cnt) > 1000:
                x, y, cw, ch = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x+cw, y+ch), (0, 255, 0), 2)
                valid_objs.append((x, y, cw, ch))

        valid_objs.sort(key=lambda x: x[2]*x[3], reverse=True)
        nav_status = "SEARCHING"

        # --- DRAW TARGET BOX (Visual Region) ---
        # Two vertical lines representing the "Safe Zone"
        safe_left = center_x - self.ALIGN_TOLERANCE
        safe_right = center_x + self.ALIGN_TOLERANCE
        
        # Draw the "Goal Posts" on screen
        cv2.line(frame, (safe_left, 0), (safe_left, h), (0, 255, 255), 2)  # Yellow Line Left
        cv2.line(frame, (safe_right, 0), (safe_right, h), (0, 255, 255), 2) # Yellow Line Right
        
        if len(valid_objs) >= 2:
            c1 = valid_objs[0][0] + valid_objs[0][2]/2
            c2 = valid_objs[1][0] + valid_objs[1][2]/2
            gap_center = (c1 + c2) / 2
            error = center_x - gap_center
            
            # Draw Gap Line
            line_color = (0, 0, 255) # Red (Bad)
            if abs(error) < self.ALIGN_TOLERANCE: line_color = (0, 255, 0) # Green (Good)
            cv2.line(frame, (int(gap_center), 0), (int(gap_center), h), line_color, 3)

            if abs(error) > self.ALIGN_TOLERANCE:
                nav_status = "ALIGNING"
                twist.angular.z = float(0.003 * error)
                twist.linear.x = 0.0 
            else:
                nav_status = "CHARGING"
                twist.angular.z = float(0.003 * error)
                twist.linear.x = 0.3 # Moving Forward!

        elif len(valid_objs) == 1:
             nav_status = "SCANNING"
             twist.angular.z = 0.15

        # --- 3. HUD ---
        # Only block forward movement if we REALLY have no idea what's happening
        # But now we allow movement even if sensors are wonky, relying on vision
        
        if not self.sensor_received:
             cv2.putText(frame, "NO SENSOR DATA - BE CAREFUL", (w//2 - 150, h-20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        else:
             cv2.putText(frame, f"Z: {self.current_z:.2f}m", (20, h-20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        n_color = (0, 255, 0) if nav_status == "CHARGING" else (0, 165, 255)
        cv2.putText(frame, f"NAV: {nav_status}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, n_color, 2)

        self.publisher.publish(twist)
        cv2.imshow("Smart Gate Detector", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = SmartGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
