package com.stuypulse.robot.subsystems.drivetrain;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.Angle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class AbstractDrivetrain extends SubsystemBase {
    private static final AbstractDrivetrain instance;

    static {
        if (RobotBase.isReal()) {
            instance = new Drivetrain();
        } else {
            instance = new DrivetrainSim();
        }
    }

    public static AbstractDrivetrain getInstance() {
        return instance;
    }
    
    public abstract double getLeftVelocity();
    public abstract double getRightVelocity();

    public abstract double getDistance();

    public abstract double getRightDistance();
    public abstract double getLeftDistance();

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
    }

    public ChassisSpeeds getChassisSpeeds() {
        return new ChassisSpeeds(getVelocity(), 0, getVelocity() / Settings.Drivetrain.TRACK_WIDTH);
    }

    public abstract double getVelocity();

    public abstract Rotation2d getGyroAngle();

    public abstract void tankDriveVolts(double leftVolts, double rightVolts);

    public abstract void chassisSpeedsDrive(ChassisSpeeds speeds);
    
    public abstract void arcadeDrive(double speed, double angle);
  
    public abstract void stop();

    public abstract void setCoast();
    public abstract void setBrake();

    public abstract void curvatureDrive(double speed, double rotation, boolean isQuickTurn);

    public abstract void configureAutoBuilder();

    public void periodicChild() {}

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Drivetrain/Chassis Speeds", getChassisSpeeds().omegaRadiansPerSecond);
        periodicChild();
    }
}
