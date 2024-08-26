import asyncio
from multiprocessing import ProcessError
import struct
import sys
from typing import Dict
#from importlib_metadata import csv
import keyboard

import bleak
from bleak import BleakClient
from bleak import BleakScanner

FINGER_CHARAC_ID = "13012F01-F8C3-4F4A-A8F4-15CD926DA146"

class BLEClient(object):
    
    def __init__(self, uuid:str, csvout:bool=False) -> None:
        super().__init__()
        self._client = None
        self._device = None
        self._connected = False
        self._running = False
        self._uuid = uuid
        self._found = False
        self._data = {"finger_1":0, "finger_2":0, "finger_3":0, "finger_4":0}
        self._csvout = csvout 
        self.newdata = False
        self.printdata = True
    
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
        
        # Currently not connected.
        print('Arduino Nano IMU Service')
        print('Looking for Peripheral Device...')
        devices = await BleakScanner.discover()                    
        for d in devices:       
            if "Arduino NANO 33 IoT" == d.name:
                self._found = True
                self._device = d
                sys.stdout.write(f'Found Peripheral Device {self._device.address}. ')
                break
        
        # Connect and do stuff.
        async with BleakClient(d.address) as self._client:
            sys.stdout.write(f'Connected.\n')
            self._connected = True
            # Start getting data.
            await self.start()
            # Run while connected.
            while self._connected:
                if self._running:
                    # Print data.                                                           
                    if self.printdata and self.newdata:                        
                        self.print_newdata()
                        self.newdata = False
                    await asyncio.sleep(0)
    
    async def disconnect(self) -> None:
        if self._connected:
            # Stop notification first.
            self._client.stop_notify()
            self._client.disconnect()
            self._connected = False
            self._running = False
    
    async def start(self) -> None:
        if self._connected:
            # Start notification
            await self._client.start_notify(self._uuid, self.newdata_hndlr)
            self._running = True
    
    async def stop(self) -> None:
        if self._running:
            # Stop notification
            await self._client.stop_notify(self._uuid)
        
    def newdata_hndlr(self, sender, data):
        unpacked_data = struct.unpack('<HHHH', data)
        self._data['finger_1'] = unpacked_data[0]
        self._data['finger_2'] = unpacked_data[1]
        self._data['finger_3'] = unpacked_data[2]
        self._data['finger_4'] = unpacked_data[3]
        self.newdata = True
    
    def print_newdata(self) -> None:
        if self._csvout:
            _str = (f"{self.data['finger_1']}, " 
                    f"{self.data['finger_2']}, " + 
                    f"{self.data['finger_3']}, " + 
                    f"{self.data['finger_4']} " )#+
        else:
            _str = ( "| "+
                    f"{self.data['finger_1']:}, " +
                    f"{self.data['finger_2']:}, " + 
                    f"{self.data['finger_3']:}, " + 
                    f"{self.data['finger_4']:} | " )#+
                    # "Gyro: " +
                    # f"{self.data['gx']:+3.3f}, " +
                    # f"{self.data['gy']:+3.3f}, " +
                    # f"{self.data['gz']:+3.3f}")
        sys.stdout.write(_str)
        sys.stdout.flush()        


async def run():
    # Create a new IMU client.
    finger_client = BLEClient(FINGER_CHARAC_ID, False)
    await finger_client.connect()
    


if __name__ == "__main__":
    # First create an object
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(run())
        print("sdgdsag")
    except KeyboardInterrupt:
        print('\nReceived Keyboard Interrupt')
    finally:
        print('Program finished')
