#!/usr/bin/env python3

import time
import sys, os

from ev3dev.ev3 import *
    
motRight = LargeMotor(OUTPUT_B);                    assert motRight.connected
motLeft = LargeMotor(OUTPUT_D);                     assert motLeft.connected
motTurret = MediumMotor(OUTPUT_C);                  assert motTurret.connected

motRight.stop()
motLeft.stop()
motTurret.stop()
