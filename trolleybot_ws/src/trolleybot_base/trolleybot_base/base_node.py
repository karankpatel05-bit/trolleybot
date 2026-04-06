import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial
import math
import time

class TrolleyBotBase(Node):
    def __init__(self):
        super().__init__('trolleybot_base')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_radius', 0.033) # meters
        self.declare_parameter('wheel_separation', 0.16) # meters
        self.declare_parameter('ticks_per_rev', 330.0) # ticks per motor revolution
        
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        
        self.meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev
        
        # Serial connection
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.get_logger().info(f"Connected to Arduino on {self.port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect: {e}")
            self.serial_conn = None
            
        # ROS 2 Topics
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for reading serial
        self.timer = self.create_timer(0.05, self.read_serial_callback) # 20 Hz
        
        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.last_time = self.get_clock().now()
        
    def cmd_vel_callback(self, msg):
        velocity_x = msg.linear.x
        velocity_th = msg.angular.z
        
        # Differential kinematics
        vel_left = velocity_x - (velocity_th * self.wheel_separation / 2.0)
        vel_right = velocity_x + (velocity_th * self.wheel_separation / 2.0)
        
        # Convert m/s back to ticks/frame (assumes 20Hz loop on Arduino)
        # 20 frames per second means velocity = delta_ticks_per_frame * 20 * meters_per_tick
        left_target_ticks = (vel_left / self.meters_per_tick) / 20.0
        right_target_ticks = (vel_right / self.meters_per_tick) / 20.0
        
        if self.serial_conn:
            cmd_str = f"{left_target_ticks:.2f},{right_target_ticks:.2f}\n"
            self.serial_conn.write(cmd_str.encode('utf-8'))

    def read_serial_callback(self):
        if not self.serial_conn:
            return
            
        while self.serial_conn.in_waiting > 0:
            try:
                line = self.serial_conn.readline().decode('utf-8').strip()
                if ',' in line:
                    left_ticks_str, right_ticks_str = line.split(',')
                    current_left_ticks = int(left_ticks_str)
                    current_right_ticks = int(right_ticks_str)
                    
                    self.update_odometry(current_left_ticks, current_right_ticks)
            except Exception as e:
                self.get_logger().debug(f"Serial read error: {e}")

    def update_odometry(self, left_ticks, right_ticks):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        # Reset detection: If jump is too large, assume ESP32 restarted
        # Using 50% of a full revolution as a threshold for a 50ms window
        if abs(left_ticks - self.prev_left_ticks) > (self.ticks_per_rev / 2.0) or \
           abs(right_ticks - self.prev_right_ticks) > (self.ticks_per_rev / 2.0):
            self.get_logger().info("Encoder jump detected (likely reset). Synchronizing ticks.")
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            self.last_time = current_time
            return

        d_left = (left_ticks - self.prev_left_ticks) * self.meters_per_tick
        d_right = (right_ticks - self.prev_right_ticks) * self.meters_per_tick
        
        # Update previous ticks for next iteration
        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks
        
        # Calculate changes in pose
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_separation
        
        if d_theta == 0:
            self.x += d_center * math.cos(self.th)
            self.y += d_center * math.sin(self.th)
        else:
            self.x += d_center * math.cos(self.th + d_theta / 2.0)
            self.y += d_center * math.sin(self.th + d_theta / 2.0)
            self.th += d_theta
            
        # Wrap heading to [-pi, pi]
        self.th = math.atan2(math.sin(self.th), math.cos(self.th))
            
        # Publish tf
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        q = quaternion_from_euler(0, 0, self.th)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)
        
        # Publish odometry
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        # Velocities
        odom.twist.twist.linear.x = d_center / dt if dt > 0 else 0.0
        odom.twist.twist.angular.z = d_theta / dt if dt > 0 else 0.0
        
        self.odom_pub.publish(odom)
        self.last_time = current_time

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = [0.0]*4
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss
    return q

def main(args=None):
    rclpy.init(args=args)
    node = TrolleyBotBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
