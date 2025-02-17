import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from zed_msgs.msg import ObjectsStamped

class SkeletonRepublisher(Node):
    def __init__(self):
        super().__init__('skeleton_republisher')
        
        # Subscribe to skeleton data
        self.skeleton_sub = self.create_subscription(
            ObjectsStamped,
            '/zed/zed_node/body_trk/skeletons',
            self.skeleton_callback,
            10)
            
        # Publish MarkerArray
        self.marker_pub = self.create_publisher(
            MarkerArray,
            'skeletons_array',
            10)

        # Define skeleton connections for BODY_38 format
        self.skeleton_connections = [
            # Torso
            [0, 1], [1, 2], [2, 3], [3, 4],  # Spine
            [4, 5], [5, 6], [6, 7],  # Neck and head
            # Arms
            [10, 11], [11, 12], [12, 13],  # Left arm
            [13, 14], [14, 15], [15, 16], [16, 17],  # Left hand
            [4, 10],  # Left shoulder connection
            [4, 11],  # Right shoulder connection
            [11, 18], [18, 19], [19, 20],  # Right arm
            [20, 21], [21, 22], [22, 23], [23, 24],  # Right hand
            # Legs
            [0, 25], [25, 26], [26, 27], [27, 28],  # Left leg
            [28, 29], [29, 30],  # Left foot
            [0, 31], [31, 32], [32, 33], [33, 34],  # Right leg
            [34, 35], [35, 36]  # Right foot
        ]
        
        self.get_logger().info('Skeleton republisher node initialized')

    def create_line_marker(self, points, color, id, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "skeleton"
        marker.id = id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.03  # Line width
        marker.color = color
        marker.points = points
        return marker

    def skeleton_callback(self, msg):
        marker_array = MarkerArray()
        
        for obj_idx, obj in enumerate(msg.objects):
            # Create a different color for each skeleton
            color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            
            # Process 3D skeleton
            skeleton = obj.skeleton_3d
            
            for conn_idx, connection in enumerate(self.skeleton_connections):
                start_idx = connection[0]
                end_idx = connection[1]
                
                # Get keypoints
                start_point = skeleton.keypoints[start_idx]
                end_point = skeleton.keypoints[end_idx]
                
                # Skip if either point is invalid (0,0,0)
                if all(v == 0 for v in start_point.kp) or all(v == 0 for v in end_point.kp):
                    continue
                
                # Create line points
                points = [
                    Point(x=start_point.kp[0], y=start_point.kp[1], z=start_point.kp[2]),
                    Point(x=end_point.kp[0], y=end_point.kp[1], z=end_point.kp[2])
                ]
                
                # Create marker
                marker_id = obj_idx * 100 + conn_idx
                marker = self.create_line_marker(points, color, marker_id, msg.header.frame_id)
                marker_array.markers.append(marker)
        
        # Publish marker array
        self.marker_pub.publish(marker_array)
        self.get_logger().debug(f'Published {len(marker_array.markers)} markers')

def main():
    rclpy.init()
    node = SkeletonRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()