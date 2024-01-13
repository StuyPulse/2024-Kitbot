/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import java.nio.file.Path;

import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    Path DEPLOY_DIRECTORY = Filesystem.getDeployDirectory().toPath();
    public interface Drivetrain {
        //TODO: ask rain for true track width currently using cad with +-0.1
        double TRACK_WIDTH = Units.inchesToMeters(23.83);
        double WHEEL_RADIUS = Units.inchesToMeters(3.0);
        double MASS_Kg = Units.lbsToKilograms(32.5462411);
        double GEARING = 8.45;
        double J_KG_METER_SQUARED = 7.19537;
        
        public interface Motion {
            DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH);

            SimpleMotorFeedforward MOTOR_FEED_FORWARD =
                    new SimpleMotorFeedforward(FeedForward.kS, FeedForward.kV, FeedForward.kA);

            double MAX_VELOCITY = 2.0;
            double MAX_ACCELERATION = 3.0;

            //TODO: remember to tune for real values 
            public interface FeedForward {
                double kS = 0.20094;
                double kV = 1.6658;
                double kA = 0.4515;
            }
            
            public interface PID {
                double kP = 1.0;
                double kI = 0;
                double kD = 0;
            }
        }

        public interface Feedforward {
            double kV = 1.6658;
            double kA = 0.4515;

            double kVAngular = 6.34 / 5.0;
            double kAAngular = 1.35 / 5.0;
        }
    }

    public interface Launcher {
        // Speeds for wheels when intaking and launching. Intake speeds are negative to run the wheels
        // in reverse
        double LAUNCH_LAUNCHER_SPEED = 1;
        double LAUNCH_FEEDER_SPEED = 1;
        double INTAKE_LAUNCHER_SPEED = -1;
        double INTAKE_FEEDER_SPEED = -.2;

        double kLauncherDelay = 1;
    }

    public interface Alignment {

        SmartNumber SPEED_ADJ_FILTER = new SmartNumber("Alignment/Speed Adj RC", 0.1);

        SmartNumber FUSION_FILTER = new SmartNumber("Alignment/Fusion RC", 0.3);
        SmartNumber DEBOUNCE_TIME = new SmartNumber("Alignment/Debounce Time", 0.15);

        //TODO: placeholder values for proper thresholds
        SmartNumber ALIGNED_THRESHOLD_X = new SmartNumber("Alignment/X Threshold", 0.08);
        SmartNumber ALIGNED_THRESHOLD_Y = new SmartNumber("Alignment/Y Threshold", 0.1);
        SmartNumber ALIGNED_THRESHOLD_ANGLE = new SmartNumber("Alignment/Angle Threshold", 5);
        
        public interface Translation {
            SmartNumber P = new SmartNumber("Alignment/Translation/kP", 1);
            SmartNumber I = new SmartNumber("Alignment/Translation/kI", 0);
            SmartNumber D = new SmartNumber("Alignment/Translation/kD", 0.0);
        }

        public interface Rotation {
            SmartNumber P = new SmartNumber("Alignment/Rotation/kP", 1);
            SmartNumber I = new SmartNumber("Alignment/Rotation/kI", 0);
            SmartNumber D = new SmartNumber("Alignment/Rotation/kD", 0);
        }
    }

    public interface Vision {
        SmartNumber CAMERA_OFFSET_X = new SmartNumber("Vision/Camera X", 0);
        SmartNumber CAMERA_OFFSET_Y = new SmartNumber("Vision/Camera Y", 0);
        SmartNumber CAMERA_OFFSET_Z = new SmartNumber("Vision/Camera Z", 0);
        
        SmartNumber CAMERA_YAW = new SmartNumber("Vision/Camera Yaw", 0);
        SmartNumber CAMERA_PITCH = new SmartNumber("Vision/Camera Pitch", 0);
    }
}