import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import argparse

parser = argparse.ArgumentParser(description='Process ROS2 transform [@FRAME] to map frame.')
parser.add_argument("from_frame", help="Reference frame")

args = parser.parse_args()

class Transform(Node):

    def __init__(self, from_fram_ref):
        super().__init__('tf2_frame_listener')

        # Reference frame
        self.from_fram_ref = from_fram_ref
        self.to_frame_rel = 'map'

        # Transform
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publisher
        if "_" in from_fram_ref : 
            self.topic_name = from_fram_ref.split("_")[0] + from_fram_ref.split("_")[-1]
        else:
            self.topic_name = from_fram_ref

        self.publisher = self.create_publisher(Pose,  self.topic_name + 'tomap', 1)
        self.timer = self.create_timer(0.20, self.on_timer)

    def on_timer(self):
        
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                self.to_frame_rel,
                self.from_fram_ref,
                now)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.to_frame_rel} to {self.from_fram_ref}: {ex}')
            return

        # Message
        pose = Pose()
        
        pose.position.x = trans.transform.translation.x
        pose.position.y = trans.transform.translation.y
        pose.position.z = trans.transform.translation.z

        pose.orientation.x = trans.transform.rotation.x
        pose.orientation.y = trans.transform.rotation.y
        pose.orientation.z = trans.transform.rotation.z
        pose.orientation.w = trans.transform.rotation.w
        
        self.publisher.publish(pose)

        print(pose.orientation)

def main():
    rclpy.init()

    transform = Transform(from_fram_ref = args.from_frame)

    rclpy.spin(transform)

    # Destroy the node explicitly
    transform.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
