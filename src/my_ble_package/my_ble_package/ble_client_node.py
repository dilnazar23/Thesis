import asyncio
import struct
from bleak import BleakClient, BleakScanner
from typing import Dict
import rclpy
from rclpy.node import Node
import serial

FINGER_CHARAC_ID = "13012F01-F8C3-4F4A-A8F4-15CD926DA146"

class BLEClient(Node):
    
    def __init__(self, uuid: str, csvout: bool = False) -> None:
        super().__init__('ble_client_node')
        self._client = None
        self._device = None
        self._connected = False
        self._running = False
        self._uuid = uuid
        self._found = False
        self._data = {"finger_1": 0, "finger_2": 0, "finger_3": 0, "finger_4": 0}
        self._csvout = csvout
        self.newdata = False
        self.printdata = True
        self.get_logger().info('BLEClient initialized')
        # Update the baud rate as required
        self.serial_port = serial.Serial('/dev/ttyACM0', 9600)  

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
                    self.send_data_mega()
                    self.newdata = False
                await asyncio.sleep(0)

    async def start(self) -> None:
        if self._connected:
            await self._client.start_notify(self._uuid, self.newdata_hndlr)
            self._running = True

    def newdata_hndlr(self, sender, data):
        unpacked_data = struct.unpack('<HHHH', data)
        self._data['finger_1'] = unpacked_data[0]
        self._data['finger_2'] = unpacked_data[1]
        self._data['finger_3'] = unpacked_data[2]
        self._data['finger_4'] = unpacked_data[3]
        self.newdata = True

    def print_newdata(self) -> None:
        _str = (f"| {self.data['finger_1']}, {self.data['finger_2']}, " 
                f"{self.data['finger_3']}, {self.data['finger_4']} |")
        self.get_logger().info(_str)
    
    def send_data_mega(self) -> None:
        send_msg = f"S,{self.data['finger_1']},{self.data['finger_2']},{self.data['finger_3']},{self.data['finger_4']},\n"
        self.serial_port.write(send_msg.encode('utf-8'))

def main(args=None):
    rclpy.init(args=args)
    ble_client = BLEClient(FINGER_CHARAC_ID, False)

    # We must run the asyncio event loop manually
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(ble_client.connect())
    except KeyboardInterrupt:
        ble_client.get_logger().info('Keyboard Interrupt')
    finally:
        ble_client.get_logger().info('Shutting down')
        rclpy.shutdown()

if __name__ == "__main__":
    main()
