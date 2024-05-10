#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np

class LidarProcessingNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('lidar_processing_node')

        # Subscriber to the LaserScan data
        self.subscriber = rospy.Subscriber('/husky/laser/scan', LaserScan, self.callback)

        # Publisher for the OccupancyGrid
        self.publisher = rospy.Publisher('/distance_map', OccupancyGrid, queue_size=10)

        # Parameters for the occupancy grid
        self.grid_size = 100  # 100x100 grid
        self.resolution = 0.1  # Each cell is 10 cm x 10 cm

        # Create an empty occupancy grid
        self.grid = OccupancyGrid()
        self.grid.info.resolution = self.resolution
        self.grid.info.width = self.grid_size
        self.grid.info.height = self.grid_size
        self.grid.info.origin.position.x = -self.grid_size*self.resolution/2
        self.grid.info.origin.position.y = -self.grid_size*self.resolution/2
        self.grid.info.origin.position.z = 0
        self.grid.info.origin.orientation.w = 1
        self.grid.data = [-1] * (self.grid_size**2)

    def callback(self, data):
        # Process each scan
        ranges = np.array(data.ranges)
        angle = data.angle_min
        angle_increment = data.angle_increment
        grid = np.zeros((self.grid_size, self.grid_size))

        # Convert polar coordinates (angle, distance) to Cartesian coordinates (x, y)
        for r in ranges:
            if np.isinf(r):
                angle += angle_increment
                continue
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            grid_x = int((x / self.resolution) + self.grid_size/2)
            grid_y = int((y / self.resolution) + self.grid_size/2)

            if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
                grid[grid_x][grid_y] = 100  # Mark as occupied

            angle += angle_increment

        # Flatten the grid and assign it to the occupancy grid message
        self.grid.data = grid.flatten().tolist()

        # Publish the occupancy grid
        self.publisher.publish(self.grid)

if __name__ == '__main__':
    try:
        node = LidarProcessingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
