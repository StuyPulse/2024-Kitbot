package com.stuypulse.robot.subsystems.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class AbstractOdometry extends SubsystemBase {

    private static final AbstractOdometry instance;

    static {
        instance = new Odometry();
    }

    public static AbstractOdometry getInstance() {
        return instance;
    }

    protected AbstractOdometry() {}

    public abstract Field2d getField();

    public abstract Pose2d getPose();

    public abstract void resetOdometery(Pose2d pose2d);

    public abstract void updateOdometry();

    public final Translation2d getTranslation() {
        return getPose().getTranslation();
    }

    public abstract Rotation2d getRotation();
}
