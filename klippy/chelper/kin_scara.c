// Inverse kinematics stepper pulse time generation
//
// Copyright (C) 2024  Erdene Luvsandorj <erdene.lu.n@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h>   // 
#include <stdlib.h> // malloc
#include <stdio.h> // fprintf
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "pyhelper.h" // errorf
#include "trapq.h" // move_get_coord
#include <stddef.h>

struct scara_stepper {
    struct stepper_kinematics sk;
    double l1, l2;  //  l1=shoulder, l2=arm 
};

/** 
    scara inverse kinematics
    l0 + l1 * cos(a) + l2 * cos(a+b) = sqrt(x*x + y*y)
    l1 * sin(a) + l2 * sin(a+b) = z
**/

static inline double
scara_calc_arm_angle_cos(double l1, double l2, double x, double y)
{
    if (l1 == 0.0)
        return 0.0;
    if (l2 == 0.0)
        return 0.0;
    return (x*x+y*y-l1*l1-l2*l2)/(2*l1*l2);
}

static inline double
scara_calc_shoulder_angle_acos(double l1, double l2, double x, double y)
{
    double cos_b = scara_calc_arm_angle_cos(l1, l2, x, y);
    double sin_b = sqrt(1 - cos_b*cos_b);
    double d = sqrt(x*x+y*y);
    double angle = asin(l2*sin_b/d)+asin(x/d);
    return angle;
}

static double
scara_stepper_bed_angle_calc(struct stepper_kinematics *sk, struct move *m, double move_time)
{
    // struct scara_stepper *fs = container_of(
    //             sk, struct scara_stepper, sk);
    // struct coord c = move_get_coord(m, move_time);
    // return asin(c.x/sqrt(c.x*c.x+c.y*c.y));
    return 0.0;
}

static double
scara_stepper_shoulder_angle_calc(struct stepper_kinematics *sk, struct move *m, double move_time)
{
    struct scara_stepper *fs = container_of(
                sk, struct scara_stepper, sk);
    struct coord c = move_get_coord(m, move_time);
    double angle = scara_calc_shoulder_angle_acos(fs->l1, fs->l2, c.x, c.y);
    // errorf("shoulder = angle=%f, x=%f, y=%f, t=%f", angle, c.x, c.y, move_time);
    return -angle;
}

static double
scara_stepper_arm_angle_calc(struct stepper_kinematics *sk, struct move *m, double move_time)
{
    struct scara_stepper *fs = container_of(
                sk, struct scara_stepper, sk);
    struct coord c = move_get_coord(m, move_time);
    struct coord c_o = m->start_pos;
    double angle = acos(scara_calc_arm_angle_cos(fs->l1, fs->l2, c.x, c.y));
    double angle_s = scara_calc_shoulder_angle_acos(fs->l1, fs->l2, c.x, c.y);
    double angle_s_o = scara_calc_shoulder_angle_acos(fs->l1, fs->l2, c_o.x, c_o.y);
    double a = angle + (angle_s - angle_s_o);
    // fprintf(stderr, "armC = angle=%f, x=%f, y=%f, t=%.9f\n", angle, c.x, c.y, move_time);
    // fprintf(stderr, "armR = angle=%f, s=%f, s_o=%f\n", a, angle_s, angle_s_o);
    return a;
    // return angle - angle_s;
    // return angle;
}

struct stepper_kinematics * __visible
scara_stepper_alloc(char type
    , double l1, double l2)
{
    struct scara_stepper *fs = malloc(sizeof(*fs));
    memset(fs, 0, sizeof(*fs));
    fs->l1 = l1;
    fs->l2 = l2;
    if (type == 'b') {
        fs->sk.calc_position_cb = scara_stepper_bed_angle_calc;
        fs->sk.active_flags = AF_Z;
    } else if (type == 's') {
        fs->sk.calc_position_cb = scara_stepper_shoulder_angle_calc;
        fs->sk.active_flags = AF_X | AF_Y;
    } else if (type == 'a') {
        fs->sk.calc_position_cb = scara_stepper_arm_angle_calc;
        fs->sk.active_flags = AF_X | AF_Y;
    }
    return &fs->sk;
}
