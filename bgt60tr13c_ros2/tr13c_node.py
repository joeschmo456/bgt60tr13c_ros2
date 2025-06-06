#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from ifxradarsdk import get_version_full
from ifxradarsdk.fmcw import DeviceFmcw
from ifxradarsdk.fmcw.types import FmcwSimpleSequenceConfig, FmcwSequenceChirp
from bgt60tr13c_ros2.msg import Complex64Array, StampedFloat32MultiArray
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

def num_rx_antennas_from_rx_mask(rx_mask):
    # popcount for rx_mask
    c = 0
    for i in range(32):
        if rx_mask & (1 << i):
            c += 1
    return c

class RadarNode(Node):
    def __init__(self):
        super().__init__('radar_tr13c_node')
        
        self.get_logger().info("Radar SDK Version: " + get_version_full())

        
        # Create Device
        self.device = DeviceFmcw()
        self.get_logger().info("Radar firmware: " + str(self.device.get_firmware_information()))
        self.get_logger().info("Sensor Information: " + str(self.device.get_sensor_information()))
        
        # set device config
        # self.config = self.device.get_simple_sequence_config()
        # self.get_logger().info("Device Defaults: " + str(self.config))
        
        self.config = FmcwSimpleSequenceConfig(
            frame_repetition_time_s=76.92e-3,  # Frame repetition time
            chirp_repetition_time_s=1000e-6,  # Chirp repetition time
            num_chirps=77,  # chirps per frame
            tdm_mimo=False,  # set True to enable MIMO mode, which is only valid for sensors with 2 Tx antennas
            chirp=FmcwSequenceChirp(
                start_frequency_Hz=59e9,  # start RF frequency, where Tx is ON
                end_frequency_Hz=61e9,  # stop RF frequency, where Tx is OFF
                sample_rate_Hz=4e6,  # ADC sample rate
                num_samples=1376,  # samples per chirp
                rx_mask=7,  # RX mask is a 4-bit, each bit set enables that RX e.g. [1,3,7,15]
                tx_mask=1,  # TX antenna mask is a 2-bit (use value 3 for MIMO)
                tx_power_level=31,  # TX power level of 31
                lp_cutoff_Hz=500000,  # Anti-aliasing filter cutoff frequency, select value from data-sheet
                hp_cutoff_Hz=80000,  # High-pass filter cutoff frequency, select value from data-sheet
                if_gain_dB=30,  # IF-gain
            ),
        )
        sequence = self.device.create_simple_sequence(self.config)
        self.device.set_acquisition_sequence(sequence)
        # self.get_logger().info("Setting Device with Config: " + str(self.config))
        # self.device.set_config(self.config)  # Sets the config AND starts data acquisition

                
        self.radar_pub = self.create_publisher(StampedFloat32MultiArray, 'radar_tr13c', 10)
        self.timer = self.create_timer(self.config.frame_repetition_time_s, self.timer_callback)  # Match radar frame rate
        
        self.count = 0
        
    def timer_callback(self):
        try:
            frame = self.device.get_next_frame()
            
            msg = StampedFloat32MultiArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'radar_frame'
            
            msg.layout.dim.append(MultiArrayDimension())
            msg.layout.dim.append(MultiArrayDimension())
            msg.layout.dim.append(MultiArrayDimension())
            
            msg.layout.dim[0].label = "Receiver"
            msg.layout.dim[0].size = num_rx_antennas_from_rx_mask(self.config.chirp.rx_mask)
            msg.layout.dim[0].stride = num_rx_antennas_from_rx_mask(self.config.chirp.rx_mask)*self.config.num_chirps*self.config.chirp.num_samples
            
            msg.layout.dim[1].label = "chirps_per_frame"
            msg.layout.dim[1].size = self.config.num_chirps
            msg.layout.dim[1].stride = self.config.chirp.num_samples*self.config.num_chirps
            
            msg.layout.dim[2].label = "samples_per_chirp"
            msg.layout.dim[2].size = self.config.chirp.num_samples
            msg.layout.dim[2].stride = self.config.chirp.num_samples
            
            msg.data = np.array(frame).flatten().tolist()
            
            self.radar_pub.publish(msg)
            self.get_logger().info(f"Published frame {self.count}: first 10 values {np.array(frame).flatten()[:10]}")
            self.count += 1
            
        except Exception as e:
            self.get_logger().error(f'Error in timer_callback: {str(e)}')
    
    def cleanup(self):
        self.device.stop_acquisition()
        self.get_logger().info("Acquisition stopped")
        self.get_logger().info("Sensor information: " + str(self.device.get_sensor_information()))

def main(args=None):
    rclpy.init(args=args)
    node = RadarNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()