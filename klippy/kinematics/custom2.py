# Code for handling the custom kinematics
#
# Copyright (C) 2024  Erdene Luvsandorj <erdene.lu.n@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import math
import stepper
import logging

class CustomConfig:
    def __init__(self, config) -> None:
        # printer config
        self.l0 = config.getfloat('l0', above=0.)
        self.l1 = config.getfloat('l1', above=0.)
        self.l2 = config.getfloat('l2', above=0.)
        self.x = config.getfloat('x')
        self.y = config.getfloat('y')
        self.z = config.getfloat('z')


class Custom2Kinematics:
    def __init__(self, toolhead, config):
        
        c = CustomConfig(config)
        self.config = c
        
        # future use
        self.printer = config.get_printer()
        
        # Setup steppers
        self.rails = []
        for type in 'bsa':
            r = stepper.LookupMultiRail(config.getsection('stepper_'+type), units_in_radians=True)
            r.setup_itersolve('custom_stepper_alloc', type.encode(), c.l0, c.l1, c.l2)
            self.rails.append(r)
        self.steppers = [s for rail in self.rails for s in rail.get_steppers()]
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)

    def get_steppers(self):
        return self.steppers
    def calc_position(self, stepper_positions):
        pass
    def set_position(self, newpos, homing_axes):
        for rail in self.rails:
            rail.set_position(newpos)
    def home(self, homing_state):
        
        # disable all motors, then move each stepper individually
        self.printer.lookup_object('stepper_enable').motor_off()
        for axis, rail in zip('xyz', self.rails):
            rail.setup_itersolve('cartesian_stepper_alloc', axis.encode())
        for axis, rail in enumerate(self.rails):
            # ignore bed homing
            if rail.get_name(True) == 'b':
                continue
            
            homepos = [None, None, None, None]
            homepos[axis] = 0
            forcepos = list(homepos)
            forcepos[axis] = -30
            homing_state.home_rails([rail], forcepos, homepos)
        
        # set kinematic and initial position
        config = self.config
        for axis, rail in enumerate(self.rails):
            rail.setup_itersolve('custom_stepper_alloc'
                    , rail.get_name(True).encode(), config.l0, config.l1, config.l2)
        toolhead = self.printer.lookup_object('toolhead')
        curpos = toolhead.get_position()
        curpos[0] = config.x
        curpos[1] = config.y
        curpos[2] = config.z
        toolhead.set_position(curpos, homing_axes=(0, 1, 2))

    def check_move(self, move):
        pass
    def get_status(self, eventtime):
        pass

def load_kinematics(toolhead, config):
    return Custom2Kinematics(toolhead, config)