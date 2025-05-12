#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import miro2 as miro

robot = miro.interface.MiRoInterface()
time.sleep(1)

robot.set_forward_speed(0.2)
robot.sleep(5)
robot.set_forward_speed(0.0)

robot.exit()