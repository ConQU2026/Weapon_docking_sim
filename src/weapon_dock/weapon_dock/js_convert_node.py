import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JsConvertNode(Node):
    def __init__(self):
        super().__init__('js_convert_node')
        
        # 对于实时控制，使用 "Best Effort" (尽力而为) 模式
        # 这意味着如果网络不好，直接丢掉旧数据，而不是重发导致延迟堆积
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            Joy, 
            'joy', 
            self.joy_callback, 
            qos_profile
        )
        
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.get_logger().info('JsConvertNode (Low Latency Mode) has been started.')
        
        self.linearx_scale = 2.0  
        self.lineary_scale = 2.0 
        self.angularz_scale = 4.0 
        self.dead_zone = 0.1  

    def joy_callback(self, msg):
        if not msg.axes and not msg.buttons:
            return

        twist = Twist()
        axes = msg.axes


        if len(axes) > 1:
            val = axes[1]
            twist.linear.x = 0.0 if abs(val) < self.dead_zone else val * self.linearx_scale
        
        if len(axes) > 0: 
            val = axes[0]
            twist.linear.y = 0.0 if abs(val) < self.dead_zone else val * self.lineary_scale

        if len(axes) > 3:
            val = axes[3]
            twist.angular.z = 0.0 if abs(val) < self.dead_zone else val * self.angularz_scale
        
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = JsConvertNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()