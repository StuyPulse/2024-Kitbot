/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import java.nio.file.Path;

import com.pathplanner.lib.path.PathConstraints;
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
        double TRACK_WIDTH = Units.inchesToMeters(23.83);
        double WHEEL_RADIUS = Units.inchesToMeters(3.0);
        double MASS_Kg = Units.lbsToKilograms(32.5462411);
        double GEARING = 8.45;
        double J_KG_METER_SQUARED = 7.19537;

        SmartNumber MAX_SPEED_PERCENT = new SmartNumber("Driver/Max Speed Percent", 0.75);
        SmartNumber MAX_TURN_PERCENT = new SmartNumber("Driver/Max Turn Percent", 0.45);

        SmartNumber SPEED_FILTER = new SmartNumber("Driver/Speed RC", 0.2);
        SmartNumber ANGLE_FILTER = new SmartNumber("Driver/Turn RC", 0.15);
        
        public interface Motion {

            PathConstraints CONSTRAINTS = new PathConstraints(2, 2, 2, 1);

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
        double LAUNCHER_SPEAKER_SPEED = 1;
        double FEEDER_SPEAKER_SPEED = 1;

        double SPEAKER_THRESHOLD_RPM = 5700;

        double LAUNCHER_INTAKE_SPEED = -1;
        double FEEDER_INTAKE_SPEED = 0;

        SmartNumber AMP_THRESHOLD_RPM = new SmartNumber("Launcher/Amp/Threshold RPM", 2250);

        SmartNumber LAUNCHER_AMP_SPEED = new SmartNumber("Launcher/Amp/Launcher Speed", 0.4);
        SmartNumber FEEDER_AMP_SPEED = new SmartNumber("Launcher/Amp/Feeder Speed", 0.4);
    }

    public interface Alignment {

        SmartNumber SPEED_ADJ_FILTER = new SmartNumber("Alignment/Speed Adj RC", 0.1);

        SmartNumber FUSION_FILTER = new SmartNumber("Alignment/Fusion RC", 0.3);
        SmartNumber DEBOUNCE_TIME = new SmartNumber("Alignment/Debounce Time", 0.15);

        //TODO: placeholder values for proper thresholds
        SmartNumber ALIGNED_THRESHOLD_X = new SmartNumber("Alignment/X Threshold", 0.08);
        SmartNumber ALIGNED_THRESHOLD_Y = new SmartNumber("Alignment/Y Threshold", 0.1);
        SmartNumber ALIGNED_THRESHOLD_ANGLE = new SmartNumber("Alignment/Angle Threshold", 5);

        SmartNumber DISTANCE_THRESHOLD = new SmartNumber("Alignment/Distance Threshold", 3);
        
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
        //OFFSET DISTANCE FROM CENTER OF ROBOT
        SmartNumber CAMERA_OFFSET_X = new SmartNumber("Vision/Camera X (m)", 0);
        SmartNumber CAMERA_OFFSET_Y = new SmartNumber("Vision/Camera Y (m)", 0.3429);
        SmartNumber CAMERA_OFFSET_Z = new SmartNumber("Vision/Camera Z (m)", 0.136525);
        
        SmartNumber CAMERA_YAW = new SmartNumber("Vision/Camera Yaw (deg)", 0);
        SmartNumber CAMERA_PITCH = new SmartNumber("Vision/Camera Pitch (deg)", 30);
    }
}