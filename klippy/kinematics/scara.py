# Code for handling the inverse kinematics
#
# Copyright (C) 2024  Erdene Luvsandorj <erdene.lu.n@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import math
import stepper
import logging

class ScaraKinematics:
    def __init__(self, toolhead, config):
        
        # printer config
        self.l1 = config.getfloat('l1', above=0.)
        self.l2 = config.getfloat('l2', above=0.)
        
        # Setup steppers
        self.steppers = []
        for type in 'sa':
            s = stepper.PrinterStepper(config.getsection('stepper_'+type), True)
            s.setup_itersolve('scara_stepper_alloc', type.encode()
                , self.l1, self.l2)
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
            self.steppers.append(s)

        # set initial position
        # self.set_position(self.get_pos(0, self.angle1, self.angle2), ())

    def get_steppers(self):
        return self.steppers
    def calc_position(self, stepper_positions):
        pass
    def set_position(self, newpos, homing_axes):
        for s in self.steppers:
            s.set_position(newpos)
    def home(self, homing_state):
        pass
    def check_move(self, move):
        pass
    def get_status(self, eventtime):
        pass
def load_kinematics(toolhead, config):
    return ScaraKinematics(toolhead, config)
