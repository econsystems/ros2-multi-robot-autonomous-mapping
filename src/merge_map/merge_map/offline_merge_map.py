# Obstacle Priority: If any map has an obstacle (100), that value will be propagated to the merged map. This ensures that an obstacle is always marked, even if other maps indicate free space.
# Free Space (-1): If both maps have no obstacle in the overlapping region (i.e., value 0 or -1), the merged map will maintain that as free space (-1).
# Handling Merged Map Values: If a map has a free space value (0 or -1), and no other map has an obstacle at that location, it will be considered free in the merged map.


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
import numpy as np

def merge_maps(maps, frame_id):
    # Create a merged map
    merged_map = OccupancyGrid()
    merged_map.header = maps[0].header
    merged_map.header.frame_id = frame_id

    # Calculate merged map boundaries
    min_x = min([map_.info.origin.position.x for map_ in maps])
    min_y = min([map_.info.origin.position.y for map_ in maps])
    max_x = max([map_.info.origin.position.x + map_.info.width * map_.info.resolution for map_ in maps])
    max_y = max([map_.info.origin.position.y + map_.info.height * map_.info.resolution for map_ in maps])

    # Set merged map info
    merged_map.info.origin.position.x = min_x
    merged_map.info.origin.position.y = min_y
    merged_map.info.resolution = min([map_.info.resolution for map_ in maps])
    merged_map.info.width = int(np.ceil((max_x - min_x) / merged_map.info.resolution))
    merged_map.info.height = int(np.ceil((max_y - min_y) / merged_map.info.resolution))
    merged_map.data = [-1] * (merged_map.info.width * merged_map.info.height)  # Initialize with free space

    # Merge the data from each map
    for map_ in maps:
        for y in range(map_.info.height):
            for x in range(map_.info.width):
                i = x + y * map_.info.width
                merged_x = int(np.floor((map_.info.origin.position.x + x * map_.info.resolution - min_x) / merged_map.info.resolution))
                merged_y = int(np.floor((map_.info.origin.position.y + y * map_.info.resolution - min_y) / merged_map.info.resolution))
                merged_i = merged_x + merged_y * merged_map.info.width

                # Handle merging logic: if one map has an obstacle (value 100), it overrides the other map
                if map_.data[i] == 100:  # Obstacle
                    merged_map.data[merged_i] = 100
                elif map_.data[i] == 0 or map_.data[i] == -1:  # Empty space or no data
                    # Keep the merged map value as is, if it was already an obstacle in a prior map
                    if merged_map.data[merged_i] == -1:
                        merged_map.data[merged_i] = map_.data[i]
                    
    return merged_map

class MergeMapNode(Node):
    def __init__(self):
        super().__init__('merge_map_node')
        self.frame_id = self.declare_parameter('frame_id', 'merge_map').get_parameter_value().string_value
        self.robot_count = self.declare_parameter('robot_count', 3).get_parameter_value().integer_value  # Default to 3 robots

        # Use a transient_local QoS to ensure the merged map is latched
        qos = QoSProfile(depth=10)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        # Publisher for the merged map
        self.publisher = self.create_publisher(OccupancyGrid, '/' + self.frame_id, qos)

        # Store maps dynamically
        self.maps = [None] * self.robot_count

        # Create subscriptions dynamically
        for i in range(self.robot_count):
            topic = f'/tb{i + 1}/map'
            self.create_subscription(
                OccupancyGrid, topic, lambda msg, idx=i: self.map_callback(msg, idx), qos
            )

    def map_callback(self, msg, index):
        self.maps[index] = msg
        self.try_merge_and_publish()

    def try_merge_and_publish(self):
        if all(self.maps):  # Ensure all maps are available
            merged_map = merge_maps(self.maps, self.frame_id)
            self.publisher.publish(merged_map)
            # self.get_logger().info('Merged map published!')

def main(args=None):
    rclpy.init(args=args)
    merge_map_node = MergeMapNode()
    rclpy.spin(merge_map_node)
    merge_map_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
