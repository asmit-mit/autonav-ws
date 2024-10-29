# Packet Format
# Adds guards and checks to ensure correct interpretation of data stream.
#
# Data Format
# -----------
# 0 | u16 | data length
# 1 |     | data
# 3 | u16 | CR LF
# 2 | u16 | fletcher16 

from .munching import *

class Serializer:
    @staticmethod
    def serialize(data) -> bytearray:
        assert(len(data) < 65536)

        packet = bytearray([])
  
        packet.extend(pu16(len(data)))
        packet.extend(data)
        packet.extend(b"\r\n")
        packet.append(0)

        #print(packet)

        return packet

    @staticmethod
    def deserialize(packet: bytearray) -> bytearray:
        ptr = 0

        len = uu16(packet[ptr:ptr+2])
        ptr += 2

        data = packet[ptr:ptr+len]

        # TODO: CRLF fletcher16

        return data
