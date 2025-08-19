import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pd_interfaces.msg import LowCmd, MotorCmd, LowState, MotorState

class MotorPublisher(Node):

    def __init__(self):
        super().__init__('motor_publisher')
        
        # 创建 LowCmd 发布器
        self.publisher = self.create_publisher(LowCmd, '/LowCmd', 10)
        # 创建 LowState 订阅器
        self.subscription = self.create_subscription(
            LowState, 
            '/LowState', 
            self.low_state_callback, 
            10
        )
        
        # 创建定时器，每秒发布一次 LowCmd 消息
        self.timer = self.create_timer(1.0, self.publish_motor_cmd)
        # 创建定时器，每 0.1 秒打印一次 MotorState 数据
        # self.print_timer = self.create_timer(0.1, self.print_motor_state2)

        # 初始化 MotorState 数据
        self.motor_state_data = []

    def publish_motor_cmd(self):
        msg = LowCmd()

        # 创建一个长度为 16 的 MotorCmd 列表
        motor_cmds = [MotorCmd() for _ in range(16)]  # 创建一个包含 16 个 MotorCmd 对象的列表
        
        # 设置每个 MotorCmd 对象的值
        for i, motor_cmd in enumerate(motor_cmds):
            motor_cmd.mode = 1
            motor_cmd.q = 0.0
            motor_cmd.dq = 0.0
            motor_cmd.tau = 0.0  # 设置 tau 从 1 到 16
            motor_cmd.kp = 0.0
            motor_cmd.kd = 20.0
            motor_cmd.tlimit = 100.0 # 0~1000 100=10%  1000=100%

        # 将 motor_cmd 列表赋值给 msg.motor_cmd
        msg.motor_cmd = motor_cmds

        # 发布消息
        self.publisher.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % str(msg))

    def low_state_callback(self, msg):
        # 存储接收到的 MotorState 数据
        self.motor_state_data = msg.motor_state
        # 仅显示第一条 MotorState 数据
        # self.get_logger().info(f"Received MotorState: {self.motor_state_data[0]}")  # 打印第一条 MotorState 数据

    def print_motor_state(self):
        if self.motor_state_data:
            # 打印存储的 MotorState 数据
            self.get_logger().info(f"MotorState: {self.motor_state_data}")  # 每 0.1 秒打印一次
    def print_motor_state2(self):
        if self.motor_state_data:
            # 打印每个 MotorState 数据的详细内容
            for i, motor_state in enumerate(self.motor_state_data):
                # 格式化每个 motor_state 的字段
                self.get_logger().info(f"MotorNum {i+1}:")
                self.get_logger().info(f"  Mode: {motor_state.mode}")
                self.get_logger().info(f"  Position (q): {motor_state.q}")
                self.get_logger().info(f"  Velocity (dq): {motor_state.dq}")
                self.get_logger().info(f"  Torque (tau): {motor_state.tau}")
                self.get_logger().info(f"  Kp: {motor_state.kp}")
                self.get_logger().info(f"  Kd: {motor_state.kd}")
                self.get_logger().info(f"  Error: {motor_state.error}")
                self.get_logger().info("---")  # 分隔线，区分每个 MotorState


def main(args=None):
    rclpy.init(args=args)
    motor_publisher = MotorPublisher()
    rclpy.spin(motor_publisher)
    motor_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
