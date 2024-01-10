/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    public interface Drivetrain {
        int CURRENT_LIMIT = 60;
    }

    public interface Launcher {
        int LAUNCHER_CURRENT_LIMIT = 80;
        int FEEDER_CURRENT_LIMIT = 80;

        // Speeds for wheels when intaking and launching. Intake speeds are negative to run the wheels
        // in reverse
        double LAUNCH_LAUNCHER_SPEED = 1;
        double LAUNCH_FEEDER_SPEED = 1;
        double INTAKE_LAUNCHER_SPEED = -1;
        double INTAKE_FEEDER_SPEED = -.2;

        double kLauncherDelay = 1;
    }
}