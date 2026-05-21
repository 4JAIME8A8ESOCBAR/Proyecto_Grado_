#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from depthai_ros_msgs.msg import SpatialDetectionArray

class PersonDetector(Node):
    def __init__(self):
        super().__init__('person_detector')
        self.subscription = self.create_subscription(
            SpatialDetectionArray,
            '/nn/spatial_detections',
            self.callback, 10)
        self.get_logger().info('Detector de personas iniciado...')

    def callback(self, msg):
        hay_persona = False
        for detection in msg.detections:
            for result in detection.results:
                if result.hypothesis.class_id == 'person' and result.hypothesis.score > 0.5:
                    hay_persona = True
                    distance = detection.bbox.center.position.z
                    self.get_logger().info(f"✅ PERSONA DETECTADA a {distance:.2f} metros")
                    break
            if hay_persona:
                break
        
        if not hay_persona:
            self.get_logger().info("❌ No hay personas detectadas")

def main(args=None):
    rclpy.init(args=args)
    node = PersonDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
