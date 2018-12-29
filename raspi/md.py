import smbus
import time

bus = smbus.SMBus(1)

DEVICE_ADDRESS = 0x10
DEVICE_REG_WaI = 0x00
DEVICE_REG_STATUS = 0x01
DEVICE_REG_MODE = 0x02
DEVICE_REG_SRH = 0x03
DEVICE_REG_SRL = 0x04
DEVICE_REG_DUTYH = 0x05
DEVICE_REG_DUTYL = 0x06
DEVICE_REG_TOPH = 0x07
DEVICE_REG_TOPL = 0x08
DEVICE_REG_ADR = 0x09
DEVICE_REG_WRITE = 0x0A

sendBytes = [0x25,0x00,0x02,0x02,0xf0,0x04,0x00]
bus.write_i2c_block_data(DEVICE_ADDRESS,DEVICE_REG_MODE,sendBytes)
time.sleep(0.1)

reg = bus.read_i2c_block_data(DEVICE_ADDRESS,DEVICE_REG_WaI,10)

print(reg)
time.sleep(2)

sendBytes2 = [0x01,0x10]
bus.write_i2c_block_data(DEVICE_ADDRESS,DEVICE_REG_DUTYH,sendBytes2)
time.sleep(0.1)

reg = bus.read_i2c_block_data(DEVICE_ADDRESS,DEVICE_REG_WaI,10)

print(reg)

time.sleep(3)
sendBytes3 = [0x02,0x00]
bus.write_i2c_block_data(DEVICE_ADDRESS,DEVICE_REG_DUTYH,sendBytes3)

time.sleep(2)
sendBytes4 = [0x27]
bus.write_i2c_block_data(DEVICE_ADDRESS,DEVICE_REG_MODE,sendBytes4)
