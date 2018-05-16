from threading import Thread
from ev3dev import *

class PIDSystem:
    def __init__(self, funcRead, funcWrite, funcStop, kp, ki, kd, tolerance):
        self.funcRead = funcRead
        self.funcWrite = funcWrite
        self.funcStop = funcStop
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.tolerance = tolerance
        self.p = 0
        self.i = 0
        self.d = 0
        self.thread = None
        self.enabled = False
        self.setpoint = 0

    def periodic(self):
        while self.enabled:
            self.onRun()
            
        self.funcStop()

    def getError(self):
        return self.funcRead() - self.setpoint

    def isOnTarget(self):
        return abs(self.getError()) <= self.tolerance

    def onRun(self):
        self.updatePID()
        self.funcWrite(self)

    def updatePID(self):
        error = self.getError()
        self.d = self.p - error
        self.p = error
        if self.isOnTarget():
            self.i = 0
        else:
            self.i += error

    def start(self):
        self.enabled = True
        self.thread = Thread(target=self.onRun)
        self.thread.start()

    def stop(self):
        self.enabled = False
