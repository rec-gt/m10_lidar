from pymodbus import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusDeviceContext, ModbusServerContext
from pymodbus.server import StartSerialServer

store = ModbusDeviceContext(
    hr=ModbusSequentialDataBlock(0, [17] * 100),
)
context = ModbusServerContext(devices=store, single=True)
identity = ModbusDeviceIdentification()
identity.VendorName = 'RGT'
identity.ProductCode = 'RGT-LIDAR'
identity.ProductName = 'RGT-LIDAR'

print("Starting Modbus RTU Server on COM port...")
StartSerialServer(
    context=context,
    identity=identity,
    port='COM22',
    baudrate=9600,
    bytesize=8,
    parity='N',
    stopbits=1,
    timeout=1,
)
