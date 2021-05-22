import rclpy
import bisect
import rj_msgs.msg

class TopicGrabber:
    def __init__(self, topic, msg_type, node, callback=None):
        self.msgs = []
        self.keys = []
        self.msg_type = msg_type

        def record_msg(msg):
            self.msgs.append(msg)

            stamp = msg.stamp.sec * 1000000000 + msg.stamp.nanosec if hasattr(msg, 'stamp') else node.get_clock().now().nanoseconds

            self.keys.append(stamp)
            if callback:
                callback(msg, stamp)

        self.sub = node.create_subscription(topic=topic, qos_profile=100, msg_type=msg_type, callback=record_msg)

    def get_index_for_time(self, time):
        index = max(0, bisect.bisect(self.keys, time) - 1)
        return index

    def get_msg_for_index(self, index):
        return self.msgs[index]

    def get_msg_for_time(self, time):
        return self.get_msg_for_index(self.get_index_for_time(time))

    def get_latest_message(self):
        return self.msgs[-1]

    def has_message(self):
        return len(self.msgs) > 0

    def latest_or_default(self):
        return self.get_latest_message() if self.has_message() else self.msg_type()

if __name__ == '__main__':
    rclpy.init()
    node = rclpy.create_node('log_control')
    grabbers = [TopicGrabber(f"/planning/trajectory/robot_{i}", rj_msgs.msg.Trajectory, node) for i in range(16)]
    rclpy.spin(node)
    rclpy.shutdown()
