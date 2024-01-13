package com.stuypulse.robot.subsystems.drivetrain;

import com.stuypulse.stuylib.math.Angle;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
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

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
    }

    public abstract double getVelocity();

    public abstract Angle getAngle();

    public abstract void tankDriveVolts(double leftVolts, double rightVolts);
    public abstract void arcadeDrive(double speed, double angle);
    // public abstract void curvatureDrive(double speed, double angle, boolean isQuickTurn);
    public abstract void stop();

    public abstract void periodicChild();

    @Override
    public void periodic() {
        periodicChild();
    }

   
        
}
