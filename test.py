import asyncio

from pymodbus import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusDeviceContext, ModbusServerContext
from pymodbus.server import StartAsyncSerialServer

identity = ModbusDeviceIdentification()
identity.VendorName = 'RGT'
identity.ProductCode = 'RGT-LIDAR'
identity.ProductName = 'RGT-LIDAR'


async def start():
    store = ModbusDeviceContext(
        hr=ModbusSequentialDataBlock(0, [17] * 100),
    )
    context = ModbusServerContext(devices=store, single=True)
    print("Starting Modbus RTU Server on COM port...")
    server = await StartAsyncSerialServer(
        context=context,
        identity=identity,
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


asyncio.run(start())
