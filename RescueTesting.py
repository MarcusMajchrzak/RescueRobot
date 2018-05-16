#!/usr/bin/env python3

import time
import sys, os
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
targetAngle = 0
targetDis = 15
targetEnc = 0

gyroOffset = 0

gyroDriftRate = 0
gyroDriftStart = 0

rotation = 0
speed = 0

#-----Constants-----#
angleTolerance = 3
disTolerance = 10

angNorth = 0
angEast = 90
angSouth = 180
angWest = 270

angCalibration = 5

spDriveSlow = 30
spDriveMed = 40
spDriveFast = 80

spTurret = 35
spClaw = 80

gyroKp = 3
encoderKp = 3

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

def setGyro(angle):
    global gyroOffset
    global gyroDriftStart

    gyroOffset = senGyro.value() - angle
    gyroDriftStart = time.time()

def getGyroError():
    return getGyro() - targetAngle

def resetEncoders():
    motLeft.position = 0
    motRight.position = 0

def getEncoderPos():
    return ((motLeft.position + motRight.position) / 2) / 360 * wheelCircumference

def getUltrasonicFront():
    return senUltrasonicFront.value()

def getDisError():
    return getUltrasonicFront() - targetDis

def getEncPosError():
    return getEncoderPos() - targetEnc

def getUltrasonicTurret():
    return senUltrasonicTurret.value() * 10

def tankDrive(l, r):
    l = min(max(l, -100), 100)
    r = min(max(r, -100), 100)
    
    motLeft.run_direct(duty_cycle_sp = l)
    motRight.run_direct(duty_cycle_sp = r)

def drive():
    tankDrive(speed + rotation, speed - rotation)

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

def isCorrectDis():
    return abs(getDisError()) <= disTolerance

def isCorrectEnc():
    return abs(getEncPosError()) <= disTolerance

def turn():
    turnConstant()

def turnConstant():
    if getGyroError() < 0:
        tankDrive(spDriveSlow, -spDriveSlow)
    else:
        tankDrive(-spDriveSlow, spDriveSlow)

def turnKp():
    tankDrive(-getGyroError() * gyroKp, getGyroError() * gyroKp)

def isCorrectHeading():
    return abs(getGyroError()) <= angleTolerance

def mapCell():
    global targetTurret
    global targetEnc
    
    options = []
    clearFront = getUltrasonicFront() > wallDisLimit

    turret.goToSetpoint(270)
    time.sleep(0.5)
    clearLeft = getUltrasonicTurret() > wallDisLimit

    turret.goToSetpoint(90)
    time.sleep(0.5)
    clearRight = getUltrasonicTurret() > wallDisLimit

    resetEncoders()
    
    targetEnc = 50
    # while not isCorrectEnc():
    #     moveEncoder()
    # stop()

    turret.goToSetpoint(270)
    time.sleep(0.5)
    clearLeft = clearLeft or getUltrasonicTurret() > wallDisLimit

    turret.goToSetpoint(90)
    time.sleep(0.5)
    clearRight = clearRight or getUltrasonicTurret() > wallDisLimit

    targetEnc = -50
    # while not isCorrectEnc():
    #     moveEncoder()
    # stop()

    turret.goToSetpoint(270)
    time.sleep(0.5)
    clearLeft = clearLeft or getUltrasonicTurret() > wallDisLimit

    turret.goToSetpoint(90)
    time.sleep(0.5)
    clearRight = clearRight or getUltrasonicTurret() > wallDisLimit
    
    targetEnc = 0
    # while not isCorrectEnc():
    #     moveEncoder()
    # stop()
    
    targetTurret = 180

    if targetAngle == angNorth:
        if clearFront:
            options.append("n")
        if clearLeft:
            options.append("w")
        if clearRight:
            options.append("e")
        options.append("s")
        
    elif targetAngle == angEast:
        if clearFront:
            options.append("e")
        if clearLeft:
            options.append("n")
        if clearRight:
            options.append("s")
        options.append("w")
        
    elif targetAngle == angSouth:
        if clearFront:
            options.append("s")
        if clearLeft:
            options.append("e")
        if clearRight:
            options.append("w")
        options.append("n")
        
    elif targetAngle == angWest:
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

def checkDis():
    stop()
    time.sleep(0.2)
    return isCorrectDis()

def moveToNextCell():
    global targetDis
    global targetEnc

    resetEncoders()

    wallCorrectionMult = 1
    
    turret.goToSetpoint(270)
    if getUltrasonicTurret() > cellLength:
        turret.goToSetpoint(90)
        wallCorrectionMult = -1

    lastSideDis = getUltrasonicTurret()
    targetEnc = cellLength

    resetEncoders()

    while not isCorrectEnc():
        newSideDis = getUltrasonicTurret()
        #print(str(newSideDis), str(getGyro()), str(senGyro.value()))
        if abs(newSideDis - cellDisLeft) < 80 and abs(newSideDis - lastSideDis) <= 20:
            if newSideDis > lastSideDis:
                setGyro(getGyro() + 5 * wallCorrectionMult)
            elif newSideDis < lastSideDis:
                setGyro(getGyro() - 5 * wallCorrectionMult)
            lastSideDis = newSideDis

        # moveEncoder()

    stop()
    cellsToWall = 0
    targetDis = cellDisFront
    
    while getDisError() > cellLength / 2:
        cellsToWall = 0
    
        while getGyro() < targetAngle + 20:
            tankDrive(spDriveSlow - 10, -spDriveSlow + 10)
            dis = getUltrasonicFront()
            cellsToWall = max(int(getUltrasonicFront() / cellLength), cellsToWall)

        while getGyro() > targetAngle - 20:
            tankDrive(-spDriveSlow + 10, spDriveSlow - 10)
            dis = getUltrasonicFront()
            cellsToWall = max(int(getUltrasonicFront() / cellLength), cellsToWall)
            
        turnToAngle(targetAngle)

        print(cellsToWall)
        targetDis = cellsToWall * cellLength + cellDisFront

    # while not isCorrectDis() or not checkDis():    
    #     move()
    # stop()

def turnToHeading(direction):
    global targetAngle

    if direction == "n":
        targetAngle = angNorth
    elif direction == "e":
        targetAngle = angEast
    elif direction == "s":
        targetAngle = angSouth
    else:
        targetAngle = angWest

    while not isCorrectHeading():
        turn()
    stop()

def turnToAngle(angle):
    global targetAngle

    targetAngle = angle

    while not isCorrectHeading():
        turn()
    stop()

def calibrateGyro():
    calibrateGyroFront()

def calibrateGyroFront():
    if getUltrasonicFront() / cellLength >= 1:
        return
    
    minDis = 3000
    minAng = getGyro()
    maxAng = getGyro()
    negVar = posVar = 0
    
    while getGyro() < targetAngle + angCalibration + posVar:
        tankDrive(spDriveSlow - 10, -spDriveSlow + 10)
        dis = getUltrasonicFront()
        ang = getGyro()
        
        #print(str(ang) + ": " + str(dis))
        
        if dis < minDis and dis > 20:
            minDis = dis
            minAng = ang
            maxAng = ang
            posVar = ang - targetAngle
        elif dis == minDis:
            maxAng = ang
            posVar = ang - targetAngle

    while getGyro() > targetAngle - angCalibration + negVar:
        tankDrive(-spDriveSlow + 10, spDriveSlow - 10)
        dis = getUltrasonicFront()
        ang = getGyro()
        
        #print(str(ang) + ": " + str(dis))
        
        if dis < minDis and dis > 20:
            minDis = dis
            minAng = ang
            maxAng = ang
            negVar = ang - targetAngle
        elif dis == minDis:
            minAng = ang
            negVar = ang - targetAngle

    stop()
    
    correction = 0

    setGyro(targetAngle + getGyro() - (minAng + maxAng) / 2 + correction)

    turnToAngle(targetAngle)

#-----Threads-----#
# th_claw = Thread(target=moveClaw);      runThreads[th_claw] = True
# ps = PIDSystem()
subsysRot = PIDSystem(getGyro, setRot, stopRot, 2, 0, 0.8, 1)

drivetrain = Subsystem(drive)

claw = MotorSystem(motClaw, 1.2, 0, 0.5, 5)

turret = MotorSystem(motTurret, 0.4, 1, 0.1, 3)
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
    turnToHeading("n")
    moveToNextCell()
    calibrateGyro()

def east():
    global path
    path.append(getEast())
    turnToHeading("e")
    moveToNextCell()
    calibrateGyro()

def south():
    global path
    path.append(getSouth())
    turnToHeading("s")
    moveToNextCell()
    calibrateGyro()

def west():
    global path
    path.append(getWest())
    turnToHeading("w")
    moveToNextCell()
    calibrateGyro()

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

def test():
    north()
    logPosition()
    east()
    logPosition()
    south()
    logPosition()
    west()
    logPosition()
    print(path)
    print(visited)

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

if withRobot:
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
    
    # field[path[-1][1]][path[-1][0]] = mapCellInitial()

def isRed():
    count = 0
    for _ in range(10):
        time.sleep(0.1)
        if senColour.color == 5:
            count += 1

    return count

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

while withRobot and not btn.any():
    # claw.setTarget(clawMinPos if claw.setpoint == clawMaxPos else clawMaxPos)
    # time.sleep(2)
    # turret.goToSetpoint((turret.setpoint + 180) % 360)
    # time.sleep(0.5)
    pass

for t in subsystems:
    t.stop()

def areSubsystemsAlive():
    res = False
    for t in subsystems:
        if t.thread.is_alive():
            res = True

    return res

while areSubsystemsAlive():
    pass
stop()