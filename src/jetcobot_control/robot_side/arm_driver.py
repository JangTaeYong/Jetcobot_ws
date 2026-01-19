import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool, Int32
from pymycobot.mycobot280 import MyCobot280

class JetCobotFullDriver(Node):
    def __init__(self):
        super().__init__('arm_driver')
        try:
            self.mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
            self.mc.thread_lock = True
            self.mc.power_on()
            self.get_logger().info('★★★ JetCobot Online (Domain 60) ★★★')
        except Exception as e:
            self.get_logger().error(f'Connect Error: {e}')

        # 구독자들
        self.create_subscription(Float64MultiArray, 'target_angles', self.angle_cb, 10)
        self.create_subscription(Bool, 'servo_status', self.servo_cb, 10)
        self.create_subscription(Int32, 'gripper_control', self.gripper_cb, 10)
        
        # 엔코더 발행자 및 타이머
        self.pub_current = self.create_publisher(Float64MultiArray, 'current_angles', 10)
        self.timer = self.create_timer(0.1, self.timer_cb)

    def servo_cb(self, msg):
        if msg.data: self.mc.power_on(); self.get_logger().info('Servo ON')
        else: self.mc.release_all_servos(); self.get_logger().info('Servo OFF')

    def angle_cb(self, msg):
        if len(msg.data) == 6:
            self.mc.send_angles(list(msg.data), 50)

    def gripper_cb(self, msg):
        # 1: Close, 0: Open (모델에 따라 0/1 반대일 수 있음)
        state = 1 if msg.data == 1 else 0 
        self.mc.set_gripper_state(state, 50)
        self.get_logger().info(f'Gripper Move: {state}')

    def timer_cb(self):
        angles = self.mc.get_angles()
        if angles and len(angles) == 6:
            msg = Float64MultiArray()
            msg.data = [float(a) for a in angles]
            self.pub_current.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JetCobotFullDriver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()