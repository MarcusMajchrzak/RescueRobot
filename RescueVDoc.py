#!/usr/bin/env python3

import time
import sys, os
from Subsystems import MotorSystem

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
    claw = MotorSystem(motClaw, 1.2, 0, 0.5, 5)

#-----Control Setpoints-----#
foundVictim = False

targetAngle = 0
targetDis = 15
targetTurret = 180
targetEnc = 0

gyroOffset = 0

gyroDriftRate = 0
gyroDriftStart = 0

#-----Constants-----#
angleTolerance = 3
disTolerance = 10
turretTolerance = 3

angNorth = 0
angEast = 90
angSouth = 180
angWest = 270

angCalibration = 5

spDriveSlow = 30
spDriveMed = 45
spDriveFast = 80

spTurret = 35

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
    motTurret.position = 0
    resetEncoders()
    motClaw.position = 0
    claw.start()

def getDriftCorrection():
    return -(time.time() - gyroDriftStart) * gyroDriftRate

def getGyro():
    return senGyro.value() - gyroOffset

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

def getEncoderError():
    return motLeft.position - motRight.position

def getTurret():
    return motTurret.position + 180

def getUltrasonicFront():
    return senUltrasonicFront.value()

def getDisError():
    return getUltrasonicFront() - targetDis

def getEncPosError():
    return getEncoderPos() - targetEnc

def getEncoderPos():
    us = getUltrasonicFront()

    if us < cellLength * 2 / 3:
        return targetEnc + cellDisFront - us
    return ((motLeft.position + motRight.position) / 2) / 360 * wheelCircumference

def getUltrasonicTurret():
    return senUltrasonicTurret.value() * 10

def tankDrive(l, r):
    l = min(max(l, -100), 100)
    r = min(max(r, -100), 100)
    
    motLeft.run_direct(duty_cycle_sp = l)
    motRight.run_direct(duty_cycle_sp = r)

def stop():
    tankDrive(0, 0)

def stopTurret():
    motTurret.run_direct(duty_cycle_sp = 0)

def stopAll():
    stop()
    stopTurret()

def driveStraight(speed):
    driveStraightGyro(speed)

def driveStraightGyro(speed):
    tankDrive(speed - getGyroError() * gyroKp, speed + getGyroError() * gyroKp)

def driveStraightEncoder(speed):
    tankDrive(speed - getEncoderError() * encoderKp, speed + getEncoderError() * encoderKp)

def move():
    if getDisError() < 0:
        driveStraight(-spDriveSlow)
    else:
        driveStraight(spDriveSlow)

def moveEncoder():
    if getEncPosError() < 0:
        driveStraight(spDriveMed)
    else:
        driveStraight(-spDriveMed)

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

def spinTurret():
    if getTurret() < targetTurret:
        motTurret.run_direct(duty_cycle_sp = spTurret)
    else:
        motTurret.run_direct(duty_cycle_sp = -spTurret)

def isCorrectTurret():
    return abs(getTurret() - targetTurret) <= turretTolerance

def spinTurretToPos(angle):
    global targetTurret
    
    targetTurret = angle
    while not isCorrectTurret():
        spinTurret()
    stopTurret()

def testCellReadings():    
    distances = {}
    distances["front"] = getUltrasonicFront()

    spinTurretToPos(270)
    time.sleep(0.5)
    distances["left"] = getUltrasonicTurret()

    spinTurretToPos(90)
    time.sleep(0.5)
    distances["right"] = getUltrasonicTurret()

    spinTurretToPos(180)
    time.sleep(0.5)
    distances["back"] = getUltrasonicTurret()

    return distances

def mapCell():
    global targetTurret
    global targetEnc
    
    options = []
    clearFront = getUltrasonicFront() > wallDisLimit

    spinTurretToPos(270)
    time.sleep(0.5)
    clearLeft = getUltrasonicTurret() > wallDisLimit

    spinTurretToPos(90)
    time.sleep(0.5)
    clearRight = getUltrasonicTurret() > wallDisLimit
    
    clearFront = clearFront or getUltrasonicFront() > wallDisLimit

    spinTurretToPos(285)
    time.sleep(0.5)
    clearLeft = clearLeft or getUltrasonicTurret() > wallDisLimit

    spinTurretToPos(105)
    time.sleep(0.5)
    clearRight = clearRight or getUltrasonicTurret() > wallDisLimit

    resetEncoders()

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

    spinTurretToPos(270)
    time.sleep(0.2)
    clearLeft = getUltrasonicTurret() > wallDisLimit

    spinTurretToPos(180)
    time.sleep(0.2)
    clearBack = getUltrasonicTurret() > wallDisLimit

    spinTurretToPos(90)
    time.sleep(0.2)
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
    
    spinTurretToPos(270)
    if getUltrasonicTurret() > cellLength:
        spinTurretToPos(90)
        wallCorrectionMult = -1

    lastSideDis = getUltrasonicTurret()
    targetEnc = cellLength

    resetEncoders()

    while not isCorrectEnc():
        newSideDis = getUltrasonicTurret()
        if abs(newSideDis - cellDisLeft) < 200 and abs(newSideDis - lastSideDis) <= 20 and abs(getEncoderPos() - targetEnc) > cellLength / 2:
            if newSideDis > lastSideDis:
                setGyro(getGyro() + 3.5 * wallCorrectionMult)
            elif newSideDis < lastSideDis:
                setGyro(getGyro() - 3.5 * wallCorrectionMult)
        lastSideDis = newSideDis

        moveEncoder()

    stop()
    cellsToWall = 0
    targetDis = cellDisFront
    
    while getDisError() > cellLength / 2:
        cellsToWall = 0
    
        while getGyro() < targetAngle + 20:
            tankDrive(spDriveSlow - 10, -spDriveSlow + 10)
            cellsToWall = max(int(getUltrasonicFront() / cellLength), cellsToWall)

        while getGyro() > targetAngle - 20:
            tankDrive(-spDriveSlow + 10, spDriveSlow - 10)
            cellsToWall = max(int(getUltrasonicFront() / cellLength), cellsToWall)
            
        turnToAngle(targetAngle)

        print(cellsToWall)
        targetDis = cellsToWall * cellLength + cellDisFront

    while not isCorrectDis() or not checkDis():    
        move()
    stop()

def turnToHeading(direction):
    global targetAngle

    if direction == "s":
        targetAngle = angSouth
    elif direction == "n":
        targetAngle = angNorth
    elif direction == "e":
        targetAngle = angEast
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

def east():
    global path
    path.append(getEast())
    turnToHeading("e")
    moveToNextCell()

def south():
    global path
    path.append(getSouth())
    turnToHeading("s")
    moveToNextCell()

def west():
    global path
    path.append(getWest())
    turnToHeading("w")
    moveToNextCell()

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
    Leds.set_color(Leds.LEFT, Leds.RED)
    Leds.set_color(Leds.RIGHT, Leds.RED)

    while not btn.any():
        pass

    Leds.set_color(Leds.LEFT, Leds.GREEN)
    Leds.set_color(Leds.RIGHT, Leds.GREEN)
    
    reset()

    time.sleep(1)
    
    field[path[-1][1]][path[-1][0]] = mapCellInitial()

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

# Runs repeatedly during execution
while withRobot:
    # If the robot hasn't been to this cell, it needs to map it and add the information
    # to the 2D field array
    if not lastReversed:
        field[path[-1][1]][path[-1][0]] = mapCell()        

    # Gets the options from the field array for the current cell, removing any that the
    # robot has already traversed
    options = getOptions()

    # Controls robot movement, choosing a direction based on a list of cardinal priorities.
    # Using cardinal directions instead of relative directions (such as left and right)
    # usually causes the robot to traverse loops without ever having to backtrack
    if "n" in options:
        north()
        lastReversed = False
    elif "s" in options:
        south()
        lastReversed = False
    elif "e" in options:
        east()
        lastReversed = False
    elif "w" in options:
        west()
        lastReversed = False
    else:
        # Retraces the last movement, by checking the coordinates of the last location and
        # determining the direction it needs to go in to return to that position
        retrace()
        lastReversed = True
        del path[-1] # Removes retrace from path
        del path[-1] # Removes movement to dead-end from path

    # Records the cell as having been visited, so that it doesn't explore it again
    logPosition()

    red = isRed()

    while red >= 3 and not foundVictim:
        if red >= 8:
            Leds.set_color(Leds.LEFT, Leds.AMBER)
            Leds.set_color(Leds.RIGHT, Leds.AMBER)
            
            # Sets the claw position to grab the victim
            # The claw MotorSystem will keep it in position
            claw.setTarget(clawMinPos)
            time.sleep(0.5)

            # While the robot isn't in its initial position, retrace steps
            # This works the same as retracing its steps during the DFS stage
            while len(path) > 0:
                retrace()
                del path[-1]
                del path[-1]

            exit()
        else:
            red = isRed()