import os
import struct
import sys
import time

class Strategy(object) : 
    _pipeSet = {}
    def __init__(self,replyPipe,requestPipe):
        if Strategy._pipeSet.has_key((replyPipe,requestPipe)) is False:
            Strategy._pipeSet[(replyPipe,requestPipe)] = {}
            Strategy._pipeSet[(replyPipe,requestPipe)]["Reply"] =  open(replyPipe,"r")
            Strategy._pipeSet[(replyPipe,requestPipe)]["Request"] =  open(requestPipe,"w")
        self.request = Strategy._pipeSet[(replyPipe,requestPipe)]["Request"]
        self.reply= Strategy._pipeSet[(replyPipe,requestPipe)]["Reply"]

    def Decision(self,targetDirection):
        targetDirection = (360 - targetDirection) % 360
        self.request.write(struct.pack("HH",targetDirection,0))
        self.request.flush()
        pointFmt = "HHH"
        (quality,angle,distance) = struct.unpack(pointFmt,self.reply.read(struct.calcsize(pointFmt)))
        angle = (360 - angle) %360;
        return (distance / 1000.0,angle)

pid = os.fork()
if pid == 0:
    os.execl("./ultra_simple","ultra_simple","/dev/ttyUSB1","1500","3000","")
    exit(0)

strategy = Strategy("./Reply","./Request")

if __name__ == "__main__" :
    while True:
        print strategy.Decision(0)
        time.sleep(1)

