package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartNumber;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.util.Fiducial;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public interface Field {
    double WIDTH = 16.54;
    double HEIGHT = 8.02;

    public static final double FIDUCIAL_SIZE = 0.15716;

    Fiducial TAGS[] = {
        new Fiducial(1,new Pose3d(new Translation3d(0, 0, Units.inchesToMeters(30)), new Rotation3d(Units.degreesToRadians(90),Units.degreesToRadians(0),Units.degreesToRadians(0)))),
        // new Fiducial(1,new Pose3d(new Translation3d(WIDTH, (1 + Units.inchesToMeters(28.125)), Units.inchesToMeters(30)), new Rotation3d(Units.degreesToRadians(90),Units.degreesToRadians(0),Units.degreesToRadians(180))))
        // new Fiducial(0,new Pose3d(new Translation3d(WIDTH, 0, Units.inchesToMeters(61.5)), new Rotation3d(Units.degreesToRadians(90),Units.degreesToRadians(0),Units.degreesToRadians(180)))), // home
        // new Fiducial(1,new Pose3d(new Translation3d(WIDTH, Units.inchesToMeters(33), Units.inchesToMeters(61.5)), new Rotation3d(Units.degreesToRadians(90),Units.degreesToRadians(0),Units.degreesToRadians(180)))),
        // new Fiducial(0,new Pose3d(new Translation3d(8.23, 1, Units.inchesToMeters(30)), new Rotation3d(Units.degreesToRadians(0),Units.degreesToRadians(0),Units.degreesToRadians(180)))), // flipped test
        // new Fiducial(1,new Pose3d(new Translation3d(8.23, 1 + Units.inchesToMeters(28.125), Units.inchesToMeters(30)), new Rotation3d(Units.degreesToRadians(0),Units.degreesToRadians(0),Units.degreesToRadians(180))))
    };

    public static boolean isValidTag(int id) {
        for (Fiducial tag : TAGS)
            if (tag.getID() == id) return true;
        return false;
    }

    public static Fiducial getTag(int id) {
        for (Fiducial tag : TAGS)
            if (tag.getID() == id) return tag;
        return null;
    }

    public static double[] getTagLayout(Fiducial[] fiducials) {
        double[] layout = new double[fiducials.length * 7];

        for (int i = 0; i < fiducials.length; i++) {
            Fiducial tag = fiducials[i];
            layout[i * 7 + 0] = tag.getID();
            layout[i * 7 + 1] = tag.getPose().getTranslation().getX();
            layout[i * 7 + 2] = tag.getPose().getTranslation().getY();
            layout[i * 7 + 3] = tag.getPose().getTranslation().getZ();
            layout[i * 7 + 4] = tag.getPose().getRotation().getX();
            layout[i * 7 + 5] = tag.getPose().getRotation().getY();
            layout[i * 7 + 6] = tag.getPose().getRotation().getZ();
        }

        return layout;
    }

    public static boolean isValidAprilTagId(int id) {
        return id >= 1 && id <= 16;
    }
}
