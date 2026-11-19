"""Reuse existing ZED node unchanged; align XYZ publication with left-frame ticks."""
import rclpy
from auv_sensing.zed_sensor_node import ZedSensorNode


class SyncedZed(ZedSensorNode):
    def _due(self,last_pub_ns,key,ts_ns,rate_hz):
        if key=='points' and self._p['publish_left_image']:
            return last_pub_ns.get('left')==ts_ns
        return super()._due(last_pub_ns,key,ts_ns,rate_hz)


def main(args=None):
    rclpy.init(args=args);node=SyncedZed()
    node.get_logger().info('Vision profile: SDK XYZ shares each left-image tick; original converters/IMU preserved')
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
