package com.stuypulse.robot.subsystems.odometry;

import com.stuypulse.robot.subsystems.drivetrain.AbstractDrivetrain;
import com.stuypulse.robot.subsystems.drivetrain.Drivetrain;
import com.stuypulse.robot.subsystems.vision.AbstractVision;
import com.stuypulse.robot.util.Fiducial;
import com.stuypulse.robot.util.VisionData;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Odometry extends AbstractOdometry {
    
    private final DifferentialDriveOdometry odometry;

    private final Field2d field;

    public Odometry() {
        AbstractDrivetrain drivetrain = AbstractDrivetrain.getInstance();
        this.odometry = new DifferentialDriveOdometry(drivetrain.getGyroAngle(), drivetrain.getLeftDistance(), drivetrain.getRightDistance());
        resetOdometery(new Pose2d());

        this.field = new Field2d();
        SmartDashboard.putData("Field", field);
    }

    public final Rotation2d getRotation() {
        return getPose().getRotation();
    }

    @Override
    public Field2d getField() {
        return field;
    }

    @Override
    public Pose2d getPose() {
        updateOdometry();
        return odometry.getPoseMeters();
    }

    @Override 
    public void updateOdometry() {
        AbstractDrivetrain drivetrain = AbstractDrivetrain.getInstance();
        odometry.update(drivetrain.getGyroAngle(), drivetrain.getLeftDistance(), drivetrain.getRightDistance());
    }

    @Override
    public void resetOdometery(Pose2d pose2d) {
        AbstractDrivetrain drivetrain = AbstractDrivetrain.getInstance();
        odometry.resetPosition(drivetrain.getGyroAngle(), drivetrain.getLeftDistance(), drivetrain.getRightDistance(), pose2d);
    }

    @Override
    public void periodic() {
        updateOdometry();
        field.setRobotPose(odometry.getPoseMeters());
        for (VisionData result : AbstractVision.getInstance().getOutput()) {

            Fiducial primaryTag = result.getPrimaryTag();
            double distance = result.calculateDistanceToTag(primaryTag);
            SmartDashboard.putNumber("Vision/Primary Tag", primaryTag.getID());
            SmartDashboard.putNumber("Vision/Primary Tag/Distance (m)", distance);
        }

        SmartDashboard.putNumber("Odometry/X (m)", odometry.getPoseMeters().getTranslation().getX());
        SmartDashboard.putNumber("Odometry/Y (m)", odometry.getPoseMeters().getTranslation().getY());
        SmartDashboard.putNumber("Odometry/Rotation (deg)", odometry.getPoseMeters().getRotation().getDegrees());
    }    
}
