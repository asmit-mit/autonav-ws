# Byte munching utilities

from struct import pack, unpack

pu8  = lambda x : pack("<B", x)
pu16 = lambda x : pack("<H", x)
pf32 = lambda x : pack("<f", x) # IEEE 754 binary32

uu8  = lambda x : unpack("<B", x)[0]
uu16 = lambda x : unpack("<H", x)[0]
uf32 = lambda x : unpack("<f", x)[0]
