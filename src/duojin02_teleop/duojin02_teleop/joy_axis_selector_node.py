
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoyAxisSelectorNode(Node):
    """
    Joy轴选择器节点
    实现方向键优先逻辑：当方向键（轴7、6）有输入时，将其映射到摇杆轴（轴1、0）
    如果方向键没有输入，则保持摇杆原值
    """
    def __init__(self):
        super().__init__("joy_axis_selector_node")
        
        self.declare_parameter('dpad_x_axis', 7)  
        self.declare_parameter('dpad_y_axis', 6)  
        self.declare_parameter('stick_x_axis', 1) 
        self.declare_parameter('stick_y_axis', 0) 
        self.declare_parameter('deadzone', 0.1)  
        
        self.dpad_x_axis = self.get_parameter('dpad_x_axis').value
        self.dpad_y_axis = self.get_parameter('dpad_y_axis').value
        self.stick_x_axis = self.get_parameter('stick_x_axis').value
        self.stick_y_axis = self.get_parameter('stick_y_axis').value
        self.deadzone = self.get_parameter('deadzone').value
        
        self.joy_sub = self.create_subscription(
            Joy, 
            '/joy_raw', 
            self.joy_callback, 
            10
        )
        
        self.joy_pub = self.create_publisher(Joy, '/joy', 10)
        
        self.get_logger().info(
            f"Joy selector started\n"
        )

    def joy_callback(self, msg: Joy):
        output_msg = Joy()
        output_msg.header = msg.header
        output_msg.buttons = list(msg.buttons)
        output_msg.axes = list(msg.axes)
        
        max_axis = max(self.dpad_x_axis, self.dpad_y_axis, 
                       self.stick_x_axis, self.stick_y_axis)
        if len(output_msg.axes) <= max_axis:
            output_msg.axes.extend([0.0] * (max_axis + 1 - len(output_msg.axes)))
        
        dpad_x = msg.axes[self.dpad_x_axis] if self.dpad_x_axis < len(msg.axes) else 0.0
        dpad_y = msg.axes[self.dpad_y_axis] if self.dpad_y_axis < len(msg.axes) else 0.0
        
        dpad_active_x = abs(dpad_x) > self.deadzone
        dpad_active_y = abs(dpad_y) > self.deadzone
        
        if dpad_active_x:
            output_msg.axes[self.stick_x_axis] = dpad_x
        
        if dpad_active_y:
            output_msg.axes[self.stick_y_axis] = dpad_y
        
        self.joy_pub.publish(output_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoyAxisSelectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
