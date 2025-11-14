from pymodbus.client import ModbusSerialClient

client = ModbusSerialClient(port='COM3', baudrate=9600, parity='N', stopbits=1, bytesize=8)
client.connect()
result = client.read_holding_registers(0, 10, slave=1)
print(result.registers)
client.close()