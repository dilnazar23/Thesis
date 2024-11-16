import asyncio
import struct
from bleak import BleakClient, BleakScanner
from typing import Dict
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

IMU_CHARAC_ID = "13012F02-F8C3-4F4A-A8F4-15CD926DA146"

class IMUClient(Node):
    
    def __init__(self, uuid: str, csvout: bool = False) -> None:
        super().__init__('imu_client_node')
        self._client = None
        self._device = None
        self._connected = False
        self._running = False
        self._uuid = uuid
        self._found = False
        self._data = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0}
        self.publisher_ = self.create_publisher(PoseStamped, 'pose', 10)
        
        self._csvout = csvout
        self.newdata = False
        self.printdata = True
        self.get_logger().info('IMUClient initialized')

    @property
    def connected(self) -> bool:
        return self._connected
    
    @property
    def data(self) -> Dict:
        return self._data

    @property
    def uuid(self) -> str:
        return self._uuid

    @property
    def running(self) -> bool:
        return self._running
    
    @property
    def device(self):
        return self._device

    async def connect(self) -> None:
        if self._connected:
            return

        self.get_logger().info('Looking for Peripheral Device...')
        devices = await BleakScanner.discover()
        for d in devices:
            if "Arduino NANO 33 IoT" == d.name:
                self._found = True
                self._device = d
                self.get_logger().info(f'Found Peripheral Device {self._device.address}.')
                break

        if not self._found:
            self.get_logger().info('Device not found.')
            return

        async with BleakClient(self._device.address) as client:
            self._client = client
            self.get_logger().info('Connected.')
            self._connected = True
            await self.start()
            while self._connected:
                if self._running and self.newdata:
                    self.print_newdata()                    
                    self.newdata = False
                await asyncio.sleep(0)

    async def start(self) -> None:
        if self._connected:
            await self._client.start_notify(self._uuid, self.newdata_hndlr)
            self._running = True

    def newdata_hndlr(self, sender, data):
        unpacked_data = struct.unpack('<ffff', data)        
        self._data['x'] = unpacked_data[0]
        self._data['y'] = unpacked_data[1]
        self._data['z'] = unpacked_data[2]
        self._data['w'] = unpacked_data[3]
        self.newdata = True
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'imu_frame'
        pose_msg.pose.orientation.x = self._data['x']
        pose_msg.pose.orientation.y = self._data['y']
        pose_msg.pose.orientation.z = self._data['z']
        pose_msg.pose.orientation.w = self._data['w']
        self.publisher_.publish(pose_msg)
        self.get_logger().info(f'Publishing: {pose_msg}')

    def print_newdata(self) -> None:
        _str = (f"| {self.data['x']}, {self.data['y']}, " 
                f"{self.data['z']}, {self.data['w']} |")
        self.get_logger().info(_str)

def main(args=None):
    rclpy.init(args=args)
    imu_client = IMUClient(IMU_CHARAC_ID, False)

    # We must run the asyncio event loop manually
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(imu_client.connect())
    except KeyboardInterrupt:
        imu_client.get_logger().info('Keyboard Interrupt')
    finally:
        imu_client.get_logger().info('Shutting down')
        rclpy.shutdown()

if __name__ == "__main__":
    main()
