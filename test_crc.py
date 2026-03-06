import crcmod

crc8_func = crcmod.mkCrcFun(0x107, initCrc=0x00, rev=False, xorOut=0x00)
data = b"0002:ARM_MOVE_TO:x=10:y=20:z=30:"
print(hex(crc8_func(data)))
