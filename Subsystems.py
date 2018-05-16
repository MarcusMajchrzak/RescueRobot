from threading import Thread
from ev3dev import *

subsystems = []

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
        subsystems.append(self)

    def setTarget(self, setpoint):
        self.setpoint = setpoint
        self.i = 0

    def periodic(self):
        while self.enabled:
            self.onRun()
        
        self.funcStop()

    def getOutput(self):
        return self.p * self.kp + self.i * self.ki - self.d * self.kd

    def getError(self):
        return self.setpoint - self.funcRead()

    def isOnTarget(self):
        return abs(self.p) <= self.tolerance

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
            self.i += 1 if self.p > 0 else -1

    def start(self):
        self.enabled = True
        self.thread = Thread(target=self.periodic)
        self.thread.start()

    def stop(self):
        self.enabled = False

    def goToSetpoint(self, setpoint):
        self.setTarget(setpoint)
        while not self.isOnTarget():
            pass

def runMotor(motor, speed):
    speed = int(min(max(speed, -100), 100))
    motor.run_direct(duty_cycle_sp = speed)

def setMotor(motorSystem):
    runMotor(motorSystem.motor, motorSystem.getOutput())

class MotorSystem(PIDSystem):
    def __init__(self, motor, kp, ki, kd, tolerance):
        super(MotorSystem, self).__init__(self.getPos, setMotor, self.stopMotor, kp, ki, kd, tolerance)
        self.motor = motor

    def getPos(self):
        return self.motor.position

    def stopMotor(self):
        self.motor.stop()
