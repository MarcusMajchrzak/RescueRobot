from threading import Thread
from ev3dev import *

subsystems = []

class Subsystem:
    '''
    The subsystem class is used to separate the processes used by each individual
    system on the robot, such as the claw or turret, and move their execution to
    a dedicated thread.
    As a result, fine control can be maintained of all conponents of the robot
    simultaneously.
    '''
    def __init__(self, func, isThreaded):
        # Holds the thread that will run this subsystem's processes
        self.thread = None

        # Controls thread execution
        self.enabled = False

        # Points to a function to be run repeatedly during execution
        self.func = func

        # Determines whether the subsystem should be run in a separate thread
        self.isThreaded = isThreaded

        # Appends the subsystem to a list of subsystems
        # This allows for grouped enabling/disabling
        subsystems.append(self)

    def periodic(self):
        '''
        This function is executed by the subsystem's thread.
        When self.enabled is changed to false, execution will end.
        '''
        while self.enabled:
            self.func()

    def start(self):
        '''
        Starts the execution of the subsystem.
        It sets the subsystem's state to enabled, and creates a new thread.
        '''
        if self.isThreaded:        
            self.enabled = True
            self.thread = Thread(target=self.periodic)
            self.thread.start()

    def stop(self):
        '''
        Stops the execution of the subsystem, by ending the condition in periodic().
        '''
        self.enabled = False

class PIDSystem(Subsystem):
    '''
    The PIDSystem class extends the generic Subsystem class to run a PID control algorithm,
    usually for a specific motor. This allows for precise and accurate motor control by
    simply setting a desired target location in the main program.
    '''
    def __init__(self, funcRead, funcWrite, funcStop, kp, ki, kd, tolerance, isThreaded):
        super(PIDSystem, self).__init__(None, isThreaded)

        # Points to a function that returns the current value of the sensor which determines
        # the current position of this system
        self.funcRead = funcRead

        # Points to a function to be run repeatedly during execution, to which the values
        # from funcRead() will be passed
        self.funcWrite = funcWrite

        # Points to a function that will stop the movement of any motors involved after
        # the thread has finished executing
        self.funcStop = funcStop

        # PID multiplier constants
        # These determine how the motor will respond under the PID control algorithm
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # The tolerance of readings within which the subsystem will consider itself
        # to be in the right position
        self.tolerance = tolerance

        # PID variables
        # These are updated during execution and used be the PID algorithm
        self.p = 0
        self.i = 0
        self.d = 0

        # The target value of the system
        self.setpoint = 0

    def setTarget(self, setpoint):
        '''
        Sets a new target value for the system.
        This is called in the main thread to tell a motor where to move to.
        After that, the PIDSystem's internal thread takes over to manage the actual movement.
        '''
        self.p += self.setpoint - setpoint
        self.setpoint = setpoint
        self.i = 0

    def periodic(self):
        '''
        This function is executed by the subsystem's thread.
        When self.enabled is changed to false, execution will end, and funcStop() will be
        called to stop the motors involved in the subsystem.
        '''
        while self.enabled:
            self.onRun()
        
        self.funcStop()

    def getOutput(self):
        '''
        Calculates the output to the motor from the PID outputs
        '''
        return self.p * self.kp + self.i * self.ki - self.d * self.kd

    def getError(self):
        '''
        Determines how far the system is from its target
        '''
        return self.setpoint - self.funcRead()

    def isOnTarget(self):
        '''
        Determines whether the system's error is within tolerance, and the system is on target
        '''
        return abs(self.p) <= self.tolerance

    def onRun(self):
        '''
        Called repeatedly during execution.
        Updates the PID variables, then moves required components.
        '''
        self.updatePID()
        self.funcWrite(self)

    def updatePID(self):
        '''
        Handles updates to the PID variables.

        P and D have been handled conventionally:
        - P is the current error
        - D is the rate at which the error is changing   

        I is determined by sum of calls, rather than sum of error. After testing, the motors
        respond better under these conditions, due to their high initial resistance to motion.
        '''
        error = self.getError()
        self.d = self.p - error
        self.p = error
        if self.isOnTarget():
            self.i = 0
        else:
            self.i += 1 if self.p > 0 else -1

    def goToSetpoint(self, setpoint):
        '''
        Called by the main thread to move a system to a target, and wait for it to get there
        before proceeding
        '''
        self.setTarget(setpoint)
        while not self.isOnTarget():
            pass

def runMotor(motor, speed):
    '''
    Generic function used to run a motor at a desired speed
    '''
    # Limits speed range to [-100, 100]
    speed = int(min(max(speed, -100), 100))
    motor.run_direct(duty_cycle_sp = speed)

def setMotor(motorSystem):
    '''
    Generic function used to pass the output of a subsystem to a motor
    '''
    runMotor(motorSystem.motor, motorSystem.getOutput())

class MotorSystem(PIDSystem):
    '''
    MotorSystem simplifies the PIDSystem setup for systems only running single motors,
    based on their own encoder values. It is used by the claw and the turret.
    '''
    def __init__(self, motor, kp, ki, kd, tolerance):
        # Using the fact that motors are controlled almost identically, MotorSystem can
        # send a group of generic functions to the PIDSystem superclass, so that we don't
        # need to individually specify how each motor works.
        super(MotorSystem, self).__init__(self.getPos, setMotor, self.stopMotor, kp, ki, kd, tolerance, True)
        self.motor = motor

    def getPos(self):
        '''
        Gets the current position of the motor.
        '''
        return self.motor.position

    def stopMotor(self):
        '''
        Stops the motor handled by this subsystem.
        '''
        self.motor.stop()
