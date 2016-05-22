import os
import struct
import sys

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
        self.request.write(struct.pack("HH",targetDirection,0))
        self.request.flush()
        pointFmt = "HHH"
        (quality,angle,distance) = struct.unpack(pointFmt,self.reply.read(struct.calcsize(pointFmt)))
        return (angle,distance)

pid = os.fork()
if pid == 0:
    os.system("sudo ./ultra_simple")
    exit(0)

strategy = Strategy("./Reply","./Request")
print strategy.Decision(0)

