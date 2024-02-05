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
    private final AbstractDrivetrain drivetrain;

    private final Field2d field;
    private final FieldObject2d odometryPose2D;

    public Odometry() {
        this.drivetrain = AbstractDrivetrain.getInstance();
        this.odometry = new DifferentialDriveOdometry(getRotation(), drivetrain.getLeftDistance(), drivetrain.getRightDistance());
        resetOdometery(new Pose2d());

        this.field = new Field2d();
        this.odometryPose2D = field.getObject("Odometry Pose");
        SmartDashboard.putData("Field", field);
    }

    public final Rotation2d getRotation() {
        if (this.odometry == null) {
            return new Rotation2d();
        }
        else {
            return this.odometry.getPoseMeters().getRotation();
        }
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
       odometry.update(getRotation(), drivetrain.getLeftDistance(), drivetrain.getRightDistance());
    }

    @Override
    public void resetOdometery(Pose2d pose2d) {
        odometry.resetPosition(getRotation(), drivetrain.getLeftDistance(), drivetrain.getRightDistance(), pose2d);
    }

    @Override
    public void periodic() {
        updateOdometry();
        field.setRobotPose(getPose());

        SmartDashboard.putData("Odometry/Field", field);

        for (VisionData result : AbstractVision.getInstance().getOutput()) {

            Fiducial primaryTag = result.getPrimaryTag();
            double distance = result.calculateDistanceToTag(primaryTag);

            SmartDashboard.putNumber("Odometry/Primary Tag/Distance (m)", distance);
        }

        odometryPose2D.setPose(odometry.getPoseMeters());
        SmartDashboard.putNumber("Odometry/Odometry/X (m)", odometry.getPoseMeters().getTranslation().getX());
        SmartDashboard.putNumber("Odometry/Odometry/Y (m)", odometry.getPoseMeters().getTranslation().getY());
        SmartDashboard.putNumber("Odometry/Odometry/Rotation (deg)", odometry.getPoseMeters().getRotation().getDegrees());
    }    
}
