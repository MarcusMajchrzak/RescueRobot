#!/usr/bin/env python3

import time
import sys, os
import math
from asyncio import Lock
from threading import Thread
from Subsystems import *

withRobot = True

#-----Robot Initialisation-----#
if withRobot:
    from ev3dev.ev3 import *
    
    motRight = LargeMotor(OUTPUT_B);                    assert motRight.connected
    motLeft = LargeMotor(OUTPUT_D);                     assert motLeft.connected
    motTurret = MediumMotor(OUTPUT_C);                  assert motTurret.connected
    motClaw = LargeMotor(OUTPUT_A);                     assert motClaw.connected
    
    senColour = ColorSensor(INPUT_1);                   assert senColour.connected
    senGyro = GyroSensor(INPUT_2);                      assert senGyro.connected
    senUltrasonicFront = UltrasonicSensor(INPUT_3);     assert senUltrasonicFront.connected
    senUltrasonicTurret = UltrasonicSensor(INPUT_4);    assert senUltrasonicTurret.connected
    
    senGyro.mode = 'GYRO-ANG'

    btn = Button()

#-----Control Setpoints-----#
gyroOffset = 0

gyroDriftRate = 0
gyroDriftStart = 0

rotation = 0
speed = 0

stage = 0
step = 0

stepStart = time.time()

#-----Constants-----#
angNorth = 0
angEast = 90
angSouth = 180
angWest = 270

angCalibration = 3

spTurret = 35
spDrive = 45

wallDisLimit = 400

cellDisFront = 110
cellDisLeft = cellDisRight = 180
cellDisBack = 80
cellLength = 420

wheelCircumference = 170
clawMinPos = -130
clawMaxPos = 0

#-----Robot Functions-----#
def reset():
    setGyro(0)
    motTurret.position = 180
    resetEncoders()
    motClaw.position = 0

def calculateDriftCorrection():
    global gyroDriftRate

    position = []
    for _ in range(5):
        position.append(senGyro.value())
        time.sleep(1)
    position.append(senGyro.value())

    gyroDriftRate = (position[-1] - position[0]) / 5

def getDriftCorrection():
    return -(time.time() - gyroDriftStart) * gyroDriftRate

def getGyro():
    return senGyro.value() - gyroOffset + getDriftCorrection()

def getDistance():
    us = getUltrasonicFront()

    if us < cellLength * 2 / 3 and stage != 1:
        return subsysLin.setpoint + cellDisFront - us
    return getEncoderPos()

def setGyro(angle):
    global gyroOffset
    global gyroDriftStart

    gyroOffset = senGyro.value() - angle
    gyroDriftStart = time.time()

def resetEncoders():
    motLeft.position = 0
    motRight.position = 0

def getEncoderPos():
    return ((motLeft.position + motRight.position) / 2) / 360 * wheelCircumference

def getUltrasonicFront():
    return senUltrasonicFront.value()

def getUltrasonicTurret():
    return senUltrasonicTurret.value() * 10

def tankDrive(l, r):
    l = min(max(l, -100), 100)
    r = min(max(r, -100), 100)
    
    motLeft.run_direct(duty_cycle_sp = l)
    motRight.run_direct(duty_cycle_sp = r)

def revalSp(speed, system):
    if system.isOnTarget():
        return 0
    elif speed > 0:
        return spDrive
    return -spDrive
    

def drive():
    tankDrive(revalSp(speed, subsysLin) + rotation, revalSp(speed, subsysLin) - rotation)

lastSideDis = 0
def calcGyroComp():
    global lastSideDis

    if lastSideDis == 0:
        lastSideDis = getUltrasonicTurret()

    wallCorrectionMult = 1 if turret.setpoint == 270 else -1
    newSideDis = getUltrasonicTurret()

    if abs(newSideDis - cellDisLeft) < 80 and abs(newSideDis - lastSideDis) <= 20:
        if newSideDis > lastSideDis:
            setGyro(getGyro() + angCalibration * wallCorrectionMult)
        elif newSideDis < lastSideDis:
            setGyro(getGyro() - angCalibration * wallCorrectionMult)
    lastSideDis = newSideDis

def setRot(rotSys):
    global rotation
    rotation = rotSys.getOutput()

def stopRot():
    global rotation
    rotation = 0

def setSp(linSys):
    global speed
    speed = linSys.getOutput()

def stopSp():
    global speed
    speed = 0

def stop():
    tankDrive(0, 0)

def turn():
    pass

def mapCell():    
    options = []
    clearFront = getUltrasonicFront() > wallDisLimit

    turret.goToSetpoint(270)
    time.sleep(0.5)
    clearLeft = getUltrasonicTurret() > wallDisLimit

    turret.goToSetpoint(90)
    time.sleep(0.5)
    clearRight = getUltrasonicTurret() > wallDisLimit

    if subsysRot.setpoint == angNorth:
        if clearFront:
            options.append("n")
        if clearLeft:
            options.append("w")
        if clearRight:
            options.append("e")
        options.append("s")
        
    elif subsysRot.setpoint == angEast:
        if clearFront:
            options.append("e")
        if clearLeft:
            options.append("n")
        if clearRight:
            options.append("s")
        options.append("w")
        
    elif subsysRot.setpoint == angSouth:
        if clearFront:
            options.append("s")
        if clearLeft:
            options.append("e")
        if clearRight:
            options.append("w")
        options.append("n")
        
    elif subsysRot.setpoint == angWest:
        if clearFront:
            options.append("w")
        if clearLeft:
            options.append("s")
        if clearRight:
            options.append("n")
        options.append("e")

    return "".join(options)

def mapCellInitial():    
    options = []
    clearFront = getUltrasonicFront() > wallDisLimit

    turret.goToSetpoint(270)
    time.sleep(0.5)
    clearLeft = getUltrasonicTurret() > wallDisLimit

    turret.goToSetpoint(180)
    time.sleep(0.5)
    clearBack = getUltrasonicTurret() > wallDisLimit

    turret.goToSetpoint(90)
    time.sleep(0.5)
    clearRight = getUltrasonicTurret() > wallDisLimit

    if clearFront:
        options.append("n")
    if clearLeft:
        options.append("w")
    if clearRight:
        options.append("e")
    if clearBack:
        options.append("s")

    print(options)
    return "".join(options)

# def calibrateGyro():
#     calibrateGyroFront()

# def calibrateGyroFront():
#     if getUltrasonicFront() / cellLength >= 1:
#         return
    
#     minDis = 3000
#     minAng = getGyro()
#     maxAng = getGyro()
#     negVar = posVar = 0
    
#     while getGyro() < targetAngle + angCalibration + posVar:
#         # tankDrive(spDriveSlow - 10, -spDriveSlow + 10)
#         dis = getUltrasonicFront()
#         ang = getGyro()
        
#         #print(str(ang) + ": " + str(dis))
        
#         if dis < minDis and dis > 20:
#             minDis = dis
#             minAng = ang
#             maxAng = ang
#             posVar = ang - targetAngle
#         elif dis == minDis:
#             maxAng = ang
#             posVar = ang - targetAngle

#     while getGyro() > targetAngle - angCalibration + negVar:
#         # tankDrive(-spDriveSlow + 10, spDriveSlow - 10)
#         dis = getUltrasonicFront()
#         ang = getGyro()
        
#         #print(str(ang) + ": " + str(dis))
        
#         if dis < minDis and dis > 20:
#             minDis = dis
#             minAng = ang
#             maxAng = ang
#             negVar = ang - targetAngle
#         elif dis == minDis:
#             minAng = ang
#             negVar = ang - targetAngle

#     stop()
    
#     correction = 0

#     setGyro(targetAngle + getGyro() - (minAng + maxAng) / 2 + correction)

#     turnToAngle(targetAngle)

def getTurret():
    return motTurret.position

def setTurret(subsystem):
    speed = 0
    if not turret.isOnTarget():
        speed = spTurret if subsystem.getOutput() > 0 else -spTurret
    runMotor(motTurret, speed)

def stopTurret():
    motTurret.stop()

#-----Subsystems-----#
subsysLin = PIDSystem(getDistance, setSp, stopSp, 0.4, 0.4, 0.05, 10, False)
subsysRot = PIDSystem(getGyro, setRot, stopRot, 2, 0, 0.8, 3, False)

drivetrain = Subsystem(drive, True)

claw = MotorSystem(motClaw, 1.2, 0, 0.5, 5)

turret = PIDSystem(getTurret, setTurret, stopTurret, 0.5, 2, 0.1, 3, True)
turret.setTarget(180)

#-----Depth-First Search-----#
def getNorth():
    return (path[-1][0], path[-1][1] + 1)

def getEast():
    return (path[-1][0] + 1, path[-1][1])

def getSouth():
    return (path[-1][0], path[-1][1] - 1)

def getWest():
    return (path[-1][0] - 1, path[-1][1])

def north():
    global path
    path.append(getNorth())
    subsysRot.setTarget(angNorth)

def east():
    global path
    path.append(getEast())
    subsysRot.setTarget(angEast)

def south():
    global path
    path.append(getSouth())
    subsysRot.setTarget(angSouth)

def west():
    global path
    path.append(getWest())
    subsysRot.setTarget(angWest)

def canNorth():
    return not getNorth() in visited and "n" in field[path[-1][1]][path[-1][0]] and path[-1][1] < maxY

def canEast():
    return not getEast() in visited and "e" in field[path[-1][1]][path[-1][0]] and path[-1][0] < maxX

def canSouth():
    return not getSouth() in visited and "s" in field[path[-1][1]][path[-1][0]] and path[-1][1] > minY

def canWest():
    return not getWest() in visited and "w" in field[path[-1][1]][path[-1][0]] and path[-1][0] > minX

def getOptions():
    options = []

    if canNorth():
        options.append("n")
    if canEast():
        options.append("e")
    if canSouth():
        options.append("s")
    if canWest():
        options.append("w")

    return options

def logPosition():
    if len(path) > 0 and not path[-1] in visited:
        visited.append(path[-1])

def printGrid(path):
    print(" " + "-"*((maxX - minX)*2+3) + " ")
    
    for y in range(maxY, minY - 1, -1):
        row = "| "
        for x in range(minX, maxX + 1):
            if (x, y) == path[0]:
                row += "S "
            elif (x, y) == path[-1]:
                row += "F "
            elif (x, y) in path:
                row += "x "
            else:
                row += ". "
        print(row + "|")
        
    print(" " + "-"*((maxX - minX)*2+3) + " ")
    print()

path = [(10, 10)]
visited = []

#-----Initialise Blank Grid-----#
minX = 0
minY = 0

field = []
for y in range(21):
    l = []
    for x in range(21):
        l.append("u")
    field.append(l)

maxX = len(field[0]) - 1
maxY = len(field) - 1

visited.append(path[-1])

#-----Robot Testing-----#
startTime = time.time()
lastReversed = True

def isRed():
    count = 0
    for _ in range(10):
        time.sleep(0.1)
        if senColour.color == 5:
            count += 1

    return count

def areSubsystemsAlive():
    res = False
    for t in subsystems:
        if t.thread and t.thread.is_alive():
            res = True

    return res

def stopSubsystems():
    for t in subsystems:
        t.stop()

    while areSubsystemsAlive():
        pass
    stop()

def retrace():
    thisCell = path[-1]
    lastCell = path[-2]
    
    difX = thisCell[0] - lastCell[0]
    difY = thisCell[1] - lastCell[1]

    if difY == 1:
        south()
    elif difY == -1:
        north()
    elif difX == 1:
        west()
    else:
        east()

def progressStep():
    global stepStart, step

    stepStart = time.time()
    step += 1

    print(str(stage) + ":" + str(step))

def progressStage():
    global stepStart, step, stage

    stepStart = time.time()
    step = 0
    stage += 1

    print(str(stage) + ":" + str(step))

def stepTime():
    return time.time() - stepStart

while withRobot and not btn.any() and stage < 3:
    # Initial Mapping
    if stage == 0:
        reset()
        # calculateDriftCorrection()

        Leds.set_color(Leds.LEFT, Leds.RED)
        Leds.set_color(Leds.RIGHT, Leds.RED)

        # while not btn.any():
        #     pass

        Leds.set_color(Leds.LEFT, Leds.GREEN)
        Leds.set_color(Leds.RIGHT, Leds.GREEN)

        time.sleep(1)
        
        for t in subsystems:
            t.start()
    
        field[path[-1][1]][path[-1][0]] = mapCellInitial()

        progressStage()

    # Periodic
    else:
        subsysLin.onRun()
        subsysRot.onRun()

        if stage == 1:
            if isRed():
                progressStage()
            elif step == 0:
                if not lastReversed:
                    field[path[-1][1]][path[-1][0]] = mapCell()        

                options = getOptions()

                if "n" in options:
                    north()
                    lastReversed = False
                elif "e" in options:
                    east()
                    lastReversed = False
                elif "w" in options:
                    west()
                    lastReversed = False
                elif "s" in options:
                    south()
                    lastReversed = False
                else:
                    if not lastReversed:
                        printGrid(path)
                        
                    retrace()
                    lastReversed = True
                    del path[-1]
                    del path[-1]

                logPosition()

                turret.goToSetpoint(90)
                if getUltrasonicTurret() > wallDisLimit:
                    turret.goToSetpoint(270)

                progressStep()
            elif step == 1:
                if subsysRot.isOnTarget() and turret.isOnTarget():
                    resetEncoders()
                    subsysLin.setTarget(cellLength + 10)
                    progressStep()
            elif step == 2:
                disError = subsysLin.getError()

                if abs(disError) < cellLength / 3 or abs(disError) > cellLength * 2 / 3:
                    calcGyroComp()
                if subsysLin.isOnTarget():
                    progressStep()
            elif step == 3:
                if stepTime() > 2:
                    step = 0

    # if stage == 0:
    #     if isRed():
    #         progressStage()
    #     else:
    #         pass
    # elif stage == 1:
    #     if step == 0:
    #         subsysLin.setTarget(subsysLin.setpoint + 150)
    #         progressStep()
    #     elif step == 1:
    #         if subsysLin.isOnTarget():
    #             progressStep()
    #     elif step == 2:
    #         if stepTime() > 0.5:
    #             claw.setTarget(clawMinPos)
    #             progressStep()
    #     elif step == 3:
    #         if stepTime() > 2:
    #             progressStep()
    #     elif step == 4:
    #         subsysLin.setTarget(subsysLin.setpoint - 150)
    #         progressStep()
    #     elif step == 5:
    #         if subsysLin.isOnTarget():
    #             progressStage()
    # elif stage == 2:
    #     progressStage()


stopSubsystems()
