"""
@file: remote_teleop_node.py
@brief: LIMO 로봇 원격 조향 제어를 위한 Twist 퍼블리셔 노드

[기술적 구성 및 설계]
1. 제어 인터페이스 분리
   - 본 노드는 원격 환경 또는 외부 입력 인터페이스로부터 제어 명령을 수신하여
     로봇의 '/cmd_vel' 토픽으로 주행 제어 명령을 퍼블리시합니다.

2. ROS 2 표준 메시지 인터페이스 (geometry_msgs/Twist)
   - ROS 2 표준 속도 명령 메시지 타입을 사용하여 시뮬레이션 환경 및
     실제 LIMO 하드웨어 모두에 수정 없이 호환됩니다.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RemoteTeleopNode(Node):
    def __init__(self):
        # 노드 이름 설정
        super().__init__('remote_teleop_node')
        
        # 로봇 제어를 위한 속도 명령(cmd_vel) 퍼블리셔 생성
        # 큐 사이즈(QoS)를 10으로 설정하여 메시지 유실 방지
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('원격 제어 노드가 시작되었습니다. (/cmd_vel 퍼블리시 준비 완료)')

    def send_command(self, linear_x, angular_z):
        """
        로봇의 선속도(linear_x)와 각속도(angular_z)를 받아 Twist 메시지로 발행하는 함수
        """
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'명령 전송됨: 전진 속도={linear_x} m/s, 회전 속도={angular_z} rad/s')

def main(args=None):
    rclpy.init(args=args)
    node = RemoteTeleopNode()
    
    try:
        # 테스트 예시: 실행 시 로봇을 앞으로 0.5m/s로 전진시키는 명령을 1회 전송합니다.
        # 실제 적용 시에는 이 부분을 키보드 입력(input) 모듈과 결합하여 동적으로 제어합니다.
        node.send_command(0.5, 0.0)
        
        # 노드를 살아있는 상태로 유지
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('키보드 인터럽트에 의해 원격 제어 노드를 종료합니다.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()