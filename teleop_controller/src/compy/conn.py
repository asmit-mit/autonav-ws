# Serial Connection Wrapper

import serial
from .packet import Serializer
from .command import serialize_command, deserialize_command

from .munching import *

# Write serial data to a file for testing
class FileConnection:
    def __init__(self, path, serializer = Serializer):
        self.path = path
        self.serializer = serializer

    def send(self, command):
        with open(self.path, "wb") as f:
            f.write(self.serializer.serialize(serialize_command(command)))

    def receive(self) -> bytearray:
        with open(self.path, "rb") as f:
            return deserialize_command(self.serializer.deserialize(f.read()))


# Serial Port
class SerialConnection:
    def __init__(self, port,timeout = 2, serializer = Serializer):
        self.serial = serial.Serial(port, 9600,timeout=timeout)
        self.serializer = serializer

    def send(self, command):
        self.serial.write(self.serializer.serialize(serialize_command(command)))

    def receive(self):
        data = bytearray([])
        data.extend(self.serial.read(2))
        data.extend(self.serial.read(uu16(data[0:2])))
        data.extend(self.serial.read(4))
        
        return deserialize_command(self.serializer.deserialize(data))
