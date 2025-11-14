from pymodbus.server.sync import StartSerialServer
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from pymodbus.datastore import ModbusSequentialDataBlock
from pymodbus.device import ModbusDeviceIdentification
import logging

# Configure logging
logging.basicConfig()
log = logging.getLogger()
log.setLevel(logging.DEBUG)

# Define Modbus data store
# Initialize with some default values in holding registers (e.g., 0-9 set to 10, 20, ..., 100)
store = ModbusSlaveContext(
    di=ModbusSequentialDataBlock(0, [0] * 100),  # Discrete Inputs
    co=ModbusSequentialDataBlock(0, [0] * 100),  # Coils
    hr=ModbusSequentialDataBlock(0, [10 * i for i in range(10)]),  # Holding Registers
    ir=ModbusSequentialDataBlock(0, [0] * 100)   # Input Registers
)

context = ModbusServerContext(slaves=store, single=True)

# Device identification (optional)
identity = ModbusDeviceIdentification()
identity.VendorName = "Custom Modbus RTU Server"
identity.ProductCode = "PMBS"
identity.VendorUrl = "http://example.com"
identity.ProductName = "Modbus RTU Server Example"
identity.ModelName = "Modbus RTU Server"
identity.MajorMinorRevision = "1.0"

# Serial port configuration
port = '/dev/ttyUSB0'  # Replace with your serial port (e.g., COM3 on Windows)
baudrate = 9600
parity = 'N'  # No parity ('E' for Even, 'O' for Odd)
stopbits = 1
bytesize = 8

# Start the Modbus RTU server
if __name__ == "__main__":
    log.info("Starting Modbus RTU server...")
    StartSerialServer(
        context=context,
        identity=identity,
        port=port,
        baudrate=baudrate,
        parity=parity,
        stopbits=stopbits,
        bytesize=bytesize,
        timeout=1
    )