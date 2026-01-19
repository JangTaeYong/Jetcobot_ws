import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory

class MoveItBridge(Node):
    def __init__(self):
        super().__init__('moveit_bridge')
        
        # 1. [구독자] GUI로부터 목표 포즈 수신
        self.pose_sub = self.create_subscription(
            Pose,
            'goal_pose', 
            self.goal_pose_callback,
            10)
        
        # [발행자] 실물 로봇(PI) 드라이버로 궤적 전송
        self.pub_to_real_robot = self.create_publisher(
            JointTrajectory, 
            'arm_controller/joint_trajectory', 
            10)

        # 2. [액션 클라이언트] MoveGroup 서버 연결
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        
        self.get_logger().info('★★★ MoveIt Bridge Online (Ready to Move) ★★★')

    def goal_pose_callback(self, msg):
        self.get_logger().info(f'새로운 목표 좌표 수신: x={msg.position.x:.3f}, y={msg.position.y:.3f}, z={msg.position.z:.3f}')
        self.send_goal(msg)

    def send_goal(self, pose):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup 서버 응답 없음! rs_mi를 확인하세요.')
            return

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm' 
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5

        # 목표 제약 조건 설정
        pos_const = PositionConstraint()
        pos_const.header.frame_id = 'base_link'
        pos_const.link_name = 'gripper_base'
        
        bv = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.01, 0.01, 0.01] # 허용 오차 약간 늘림 (1cm)
        bv.primitives.append(primitive)
        
        target_pose_stamped = PoseStamped()
        target_pose_stamped.header.frame_id = 'base_link'
        target_pose_stamped.pose = pose
        bv.primitive_poses.append(target_pose_stamped.pose)
        
        pos_const.constraint_region = bv
        pos_const.weight = 1.0

        ori_const = OrientationConstraint()
        ori_const.header.frame_id = 'base_link'
        ori_const.link_name = 'gripper_base'
        ori_const.orientation = pose.orientation
        ori_const.absolute_x_axis_tolerance = 0.1
        ori_const.absolute_y_axis_tolerance = 0.1
        ori_const.absolute_z_axis_tolerance = 0.1
        ori_const.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos_const)
        constraints.orientation_constraints.append(ori_const)
        goal_msg.request.goal_constraints.append(constraints)

        self.get_logger().info('MoveIt 엔진에게 경로 계산 요청...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('경로 계획 거부됨')
            return
        
        self.get_logger().info('경로 계획 성공! 결과 수신 대기 중...')
        # [추가된 부분] 실행 결과를 받아오기 위한 콜백 등록
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    # [새로 추가된 함수] MoveIt의 계산 결과를 실물 로봇으로 전송
    def get_result_callback(self, future):
        result = future.result().result
        # planned_trajectory 안에 로봇이 움직일 각도들이 다 들어있습니다.
        if result.planned_trajectory.joint_trajectory.points:
            self.get_logger().info('★★★ 실물 로봇(PI)으로 궤적 데이터 전송! ★★★')
            self.pub_to_real_robot.publish(result.planned_trajectory.joint_trajectory)
        else:
            self.get_logger().warn('계산된 궤적이 비어 있습니다.')

def main(args=None):
    rclpy.init(args=args)
    node = MoveItBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()