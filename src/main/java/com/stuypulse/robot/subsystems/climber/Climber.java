package com.stuypulse.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;

public class Climber extends AbstractClimber {
        CANSparkMax leftClimber;
        CANSparkMax rightClimber;

    public Climber() {
        CANSparkMax leftClimber = new CANSparkMax(Ports.Climber.LEFTCLIMBER, MotorType.kBrushless);
        CANSparkMax rightClimber = new CANSparkMax(Ports.Climber.RIGHTCLIMBER, MotorType.kBrushless);
        
        Motors.Climber.LEFTCLIMBER.configure(leftClimber);
        Motors.Climber.RIGHTCLIMBER.configure(rightClimber);

        leftClimber.follow(rightClimber);
    }
    
    @Override
    public boolean getTopHeightReached() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getTopHeightReached'");
    }

    @Override
    public boolean getBottomHeightReached() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getBottomHeightReached'");
    }

    @Override
    public void setMotorStop() {
        leftClimber.stopMotor();
        rightClimber.stopMotor();
    }
}
