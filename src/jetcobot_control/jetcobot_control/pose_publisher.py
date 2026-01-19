import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import tf2_ros  # 좌표 변환 데이터를 읽기 위해 필요함

class PoseBroadcaster(Node):
    def __init__(self):
        super().__init__('pose_broadcaster')
        # GUI와 약속한 토픽 이름
        self.publisher_ = self.create_publisher(Pose, 'current_pose', 10)
        
        # TF 데이터를 읽어오기 위한 버퍼와 리스너 설정
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 0.1초마다 실행되는 타이머
        self.timer = self.create_timer(0.1, self.publish_current_pose)
        self.get_logger().info("Pose Broadcaster (TF Mode) Started!")

    def publish_current_pose(self):
        try:
            # base_link를 기준으로 gripper_base의 위치를 가져옴
            # 사용자님의 view_frames 결과에 맞춰 'gripper_base'로 설정했습니다.
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('base_link', 'gripper_base', now)
            
            msg = Pose()
            # 위치 데이터 (x, y, z)
            msg.position.x = trans.transform.translation.x
            msg.position.y = trans.transform.translation.y
            msg.position.z = trans.transform.translation.z
            
            # 자세 데이터 (Orientation - Quaternion)
            msg.orientation = trans.transform.rotation
            
            # 토픽 발행
            self.publisher_.publish(msg)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # 데이터를 아직 못 찾았을 때는 에러를 내지 않고 그냥 넘어감
            pass

def main(args=None):
    rclpy.init(args=args)
    node = PoseBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()