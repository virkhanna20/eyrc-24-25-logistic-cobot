import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class FixedFrameBroadcaster(Node):

    def __init__(self):

        super().__init__('sample_tf2_broadcaster')                              # node initialisation
        self.tf_broadcaster = TransformBroadcaster(self)                        # initializing transform broadcaster object
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)      # timer based function which iterates on defined time interval

    def broadcast_timer_callback(self):

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()                        # select transform time stamp as current clock time
        # frame IDs
        t.header.frame_id = 'base_link'                                         # parent frame link with whom to send transform
        t.child_frame_id = 'obj_1'                                              # child frame link from where to send transfrom
        # translation
        t.transform.translation.x = 0.0
        t.transform.translation.y = 2.0                                         # distance offset in Y axis of 2 units
        t.transform.translation.z = 0.0
        # rotation
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0                                            # rotation 0 degrees

        self.tf_broadcaster.sendTransform(t)                                    # publish transform as defined in 't'


def main():
    rclpy.init()
    node = FixedFrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

