# Code for handling the inverse kinematics
#
# Copyright (C) 2024  Erdene Luvsandorj <erdene.lu.n@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import math
import stepper
import logging

class CustomKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        self.rails = []
        for type in 'x':
            r = stepper.LookupMultiRail(config.getsection('stepper_'+type))
            r.setup_itersolve('cartesian_stepper_alloc', type.encode())
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
        pass
    def home(self, homing_state):
        # force move stepper without kinematic
        force_move = self.printer.lookup_object('force_move')
        toolhead = homing_state.toolhead
        for rail in self.rails:
            # TODO ignore bed stepper
            for stepper in rail.get_steppers():
                move_end_print_time = toolhead.get_last_move_time()
                done = 0
                for mcu_endstop, n in rail.get_endstops():
                    while 1:
                        done = mcu_endstop.query_endstop(move_end_print_time)
                        if done:
                            break
                        force_move.manual_move(stepper, -.1, 200, 200)
                        move_end_print_time = toolhead.get_last_move_time()
        # set kinematic position
        curpos = toolhead.get_position()
        curpos[0] = 0
        toolhead.set_position(curpos, homing_axes=(0, 1, 2))

    def check_move(self, move):
        pass
    def get_status(self, eventtime):
        pass

def load_kinematics(toolhead, config):
    return CustomKinematics(toolhead, config)