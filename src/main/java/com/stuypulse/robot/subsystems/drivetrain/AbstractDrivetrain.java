package com.stuypulse.robot.subsystems.drivetrain;

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

    public abstract void tankDriveVolts(double leftVolts, double rightVolts);
    public abstract void curvatureDrive(double speed, double angle, boolean isQuickTurn);
    public abstract void stop();


    @Override
    public void periodic() {
        periodicChild();
    }

    public abstract void periodicChild();
        
}
