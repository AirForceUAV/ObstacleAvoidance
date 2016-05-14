import os
import struct
import sys

#try:
#    os.mkfifo("./fifo",0777)
#except Exception:
#    pass

while 1:
    request = open("./Request","w")
    reply = open("./Reply","r")

    targetDirection = int(raw_input("Input Target Direction :   "))

    request.write(struct.pack("HH",targetDirection,0))
    print 'Write OK'
    pointFmt = "HHH"
    (quality,angle,distance) = struct.unpack(pointFmt,reply.read(struct.calcsize(pointFmt)))
    print (quality,angle,distance)
   

request.close()
reply.close()

