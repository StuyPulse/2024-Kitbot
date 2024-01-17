package com.stuypulse.robot.subsystems.climber;

import edu.wpi.first.wpilibj.RobotBase;

public abstract class AbstractClimber {
    private static final AbstractClimber instance;

    static {
        if (RobotBase.isReal()) {
            instance = new Climber();
        } else {
            instance = new Climber();
        }
    }

    public AbstractClimber getInstance() {
        return instance;
    }

    //public abstract double getPosition();
    //public abstract double getVelocity();

    
    public abstract void setMotorStop();

    public abstract boolean getTopHeightReached();
    public abstract boolean getBottomHeightReached();




}
