#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from bgt60tr13c_ros2.msg import StampedFloat32MultiArray

class RadarListener(Node):
    def __init__(self):
        super().__init__('radar_tr13c_listener')
        
        # Create subscription to radar data
        self.subscription = self.create_subscription(
            StampedFloat32MultiArray,
            'radar_tr13c',
            self.radar_callback,
            10)
        
        self.get_logger().info('Radar listener node started')
        
    def radar_callback(self, msg):
        # Get dimensions from the message
        num_rx = msg.layout.dim[0].size
        num_chirps = msg.layout.dim[1].size
        num_samples = msg.layout.dim[2].size
        
        # Convert flat data to 3D array
        data = np.array(msg.data).reshape(num_rx, num_chirps, num_samples)
        
        # Log some basic information
        self.get_logger().info(f'Received radar frame:')
        self.get_logger().info(f'  - Timestamp: {msg.header.stamp}')
        self.get_logger().info(f'  - Shape: {data.shape}')
        self.get_logger().info(f'  - First few values: {data.flatten()[:5]}')

def main(args=None):
    rclpy.init(args=args)
    listener = RadarListener()
    
    try:
        rclpy.spin(listener)
    except KeyboardInterrupt:
        pass
    finally:
        listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 