import asyncio

from pymodbus import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusDeviceContext, ModbusServerContext
from pymodbus.server import StartAsyncSerialServer


class ModbusRTUServer:
    identity = None

    def init(self):
        identity = ModbusDeviceIdentification()
        identity.VendorName = 'RGT'
        identity.ProductCode = 'RGT-LIDAR'
        identity.ProductName = 'RGT-LIDAR'

    async def loop(self):
        store = ModbusDeviceContext(
            hr=ModbusSequentialDataBlock(0, [17] * 100),  # start from 40000
        )
        context = ModbusServerContext(devices=store, single=True)
        print("Starting Modbus RTU Server on COM port...")
        server = await StartAsyncSerialServer(
            context=context,
            identity=self.identity,
            port='COM22',
            baudrate=9600,
            bytesize=8,
            parity='N',
            stopbits=1,
            timeout=1,
        )

        try:
            await asyncio.Event().wait()  # runs forever (Ctrl+C to stop)
        except KeyboardInterrupt:
            print("\nStopping server…")
        finally:
            await server.shutdown()  # clean shutdown

    def start(self):
        asyncio.run(self.loop())


modbusRTUServer = ModbusRTUServer()
modbusRTUServer.init()
modbusRTUServer.start()
print("123123123")
