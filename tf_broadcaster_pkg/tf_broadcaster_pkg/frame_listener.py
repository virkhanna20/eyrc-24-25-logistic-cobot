import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener

class FrameListener(Node):

    def __init__(self):
        super().__init__('sample_tf2_frame_listener')  # node initialization
        self.tf_buffer = Buffer()  # initializing transform buffer object
        self.tf_listener = TransformListener(self.tf_buffer, self)  # initializing transform listener object
        self.timer = self.create_timer(1.0, self.on_timer)  # call 'on_timer' function every second

    def on_timer(self):
        from_frame_rel = 'obj_1'  # frame from which transform has been sent
        to_frame_rel = 'base_link'  # frame to which transform has been sent

        try:
            t = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, rclpy.time.Time())  # lookup transformation
            self.get_logger().info('Successfully received data!')
        except TransformException as e:
            self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {e}')
            return

        # Logging transform data...
        self.get_logger().info(f'Translation X:  {t.transform.translation.x}')
        self.get_logger().info(f'Translation Y:  {t.transform.translation.y}')
        self.get_logger().info(f'Translation Z:  {t.transform.translation.z}')
        self.get_logger().info(f'Rotation X:  {t.transform.rotation.x}')  # rotations are in quaternions
        self.get_logger().info(f'Rotation Y:  {t.transform.rotation.y}')
        self.get_logger().info(f'Rotation Z:  {t.transform.rotation.z}')
        self.get_logger().info(f'Rotation W:  {t.transform.rotation.w}')

def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

