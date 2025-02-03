# Handles order requests and cancellations
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class OrderManager(Node):
    def __init__(self):
        super().__init__('order_manager')
        self.order_pub = self.create_publisher(String, '/order_request', 10)
        self.cancel_sub = self.create_subscription(String, '/cancel_order', self.cancel_callback, 10)
        self.active_orders = []

    def publish_order(self, tables):
        order_msg = String()
        order_msg.data = ",".join(tables)
        self.order_pub.publish(order_msg)
        self.active_orders.extend(tables)

    def cancel_callback(self, msg):
        if msg.data in self.active_orders:
            self.active_orders.remove(msg.data)