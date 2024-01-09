/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

/** This file contains the different ports of motors, solenoids and sensors */
public interface Ports {
    public interface Gamepad {
        int DRIVER = 0;
        int OPERATOR = 1;
        int DEBUGGER = 2;
    }

    public interface Drivetrain {
        int LEFTREAR = 1;
        int LEFTFRONT = 2;
        int RIGHTREAR = 3;
        int RIGHTFRONT = 4;
    }

    public interface Launcher {
        int FEEDER = 5;
        int LAUNCHER = 6;
    }
}