import threading

from pymodbus import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusDeviceContext, ModbusServerContext
from pymodbus.server import StartSerialServer


class ModbusRTUServer:
    server_thread = None

    identity = None

    def init(self):
        self.identity = ModbusDeviceIdentification()
        self.identity.VendorName = 'RGT'
        self.identity.ProductCode = 'RGT-LIDAR'
        self.identity.ProductName = 'RGT-LIDAR'

    def loop(self):
        while True:
            store = ModbusDeviceContext(
                hr=ModbusSequentialDataBlock(0, [17] * 100),  # start from 40000
            )
            context = ModbusServerContext(devices=store, single=True)
            print("Starting Modbus RTU Server on COM port...")
            StartSerialServer(
                context=context,
                identity=self.identity,
                port='COM22',
                baudrate=9600,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=1,
            )

    def start_server_thread(self):
        if self.server_thread is None or not self.server_thread.is_alive():
            self.server_thread = threading.Thread(target=self.loop, daemon=True)
            self.server_thread.start()
            print("[Main] Server thread started.")
        else:
            print("[Main] Server already running.")


modbusRTUServer = ModbusRTUServer()
modbusRTUServer.init()
modbusRTUServer.start_server_thread()
print("123123123")
