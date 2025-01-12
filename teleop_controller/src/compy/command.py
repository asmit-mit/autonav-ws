# Motor Controller Commands
#
# Command Format
# --------------
# | 0 | u8  | command ID
# | 1 |     | command data

from .munching import *

# Command serialization framework
def serialize_command(command) -> bytearray:
    data = bytearray([])
    data.extend(pu8(command.COMMAND))
    data.extend(command.serialize())

    return data

# Command deserialization framework
def deserialize_command(data: bytearray):
    command = data[0]

    if command == Echo.COMMAND:
        return Echo.deserialize(data[1:])
    elif command == ChangeKp.COMMAND:
        return ChangeKp.deserialize(data[1:])
    elif command == ChangeKi.COMMAND:
        return ChangeKi.deserialize(data[1:])
    elif command == ChangeRpmL.COMMAND:
        return ChangeRpmL.deserialize(data[1:])
    elif command == ChangeRpmR.COMMAND:
        return ChangeRpmR.deserialize(data[1:])
    else:
        raise Exception("unknown command")

# Echo Command
#
# Command Format
# --------------
# 0 | u8  | data len
# 1 |     | echo data
class Echo:
    COMMAND = 1

    def __init__(self, bytes: bytes):
        self.bytes = bytes

    def len(self):
        return len(self.bytes)
    
    def serialize(self) -> bytearray:
        assert(self.len() < 256)

        data = bytearray([])
        data.extend(pu8(self.len()))
        data.extend(self.bytes)

        return data
    
    @staticmethod
    def deserialize(data: bytearray):
        len = data[0]
        return Echo(bytes(data[1:1+len]))

# Generic Packed 32 bit float
#
# Command Format
# --------------
# | 0 | u8[4] | IEEE binary32 float
class SingleFloat:
    def __init__(self, value: float):
        self.value = value

    def serialize(self) -> bytearray:
        return bytearray(pf32(self.value))
    
    def deserialize(data: bytearray):
        return ChangeKp(uf32(data[:4]))

# Inherits SingleFloat format

class ChangeKp(SingleFloat):
    COMMAND = 2

class ChangeKi(SingleFloat):
    COMMAND = 3

class ChangeRpmL(SingleFloat):
    COMMAND = 4

class ChangeRpmR(SingleFloat):
    COMMAND = 5
