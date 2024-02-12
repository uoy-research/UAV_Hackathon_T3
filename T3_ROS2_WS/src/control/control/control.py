import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class Control(Node):


    def __init__(self):
        super().__init__('control')
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.t = 0
        self.twist = Twist()
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0

        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0

    def timer_callback(self):
        if self.t <1000:
            self.twist.linear.z = 100.0
            self.publisher_.publish(self.twist)
        if self.t>1000 and self.t < 1500:
            self.twist.linear.y= -2.0
            self.publisher_.publish(self.twist)
        if self.t > 1500 and self.t < 2200:
            self.twist.linear.x = 0.3
            self.twist.linear.y = -1.5
            self.twist.linear.z = 0.0
            #self.twist.angular.z = 10.0
            self.publisher_.publish(self.twist)
        if self.t > 2200 and self.t < 2700:
            self.twist.linear.x = 0.0
            self.twist.linear.y = 0.0
            self.twist.linear.z = 0.1
            self.publisher_.publish(self.twist)
        if self.t > 5000:
            self.twist.linear.z = -100.0
            self.publisher_.publish(self.twist)


        #self.get_logger().info('Twist: "%s"' % self.twist)
        self.t += 1
       


def main(args=None):
    rclpy.init(args=args)

    control = Control()

    rclpy.spin(control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
