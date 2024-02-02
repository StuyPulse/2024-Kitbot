package com.stuypulse.robot.subsystems.launcher;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {

    private static final Launcher instance;

    static {
        instance = new Launcher();
    }

    public static Launcher getInstance() {
        return instance;
    }

    private final CANSparkMax feeder;
    private final CANSparkMax launcher;
    
    public Launcher() {
        feeder = new CANSparkMax(Ports.Launcher.FEEDER, MotorType.kBrushless); 
        launcher = new CANSparkMax(Ports.Launcher.LAUNCHER, MotorType.kBrushless);

        Motors.Launcher.FEEDER.configure(feeder);
        Motors.Launcher.LAUNCHER.configure(launcher);
   
    }

    //********** SETTERS **********
    public void setLaunchSpeed(Number speed) {
        launcher.set(speed.doubleValue());
    }

    public void setFeederSpeed(Number speed) {
        feeder.set(speed.doubleValue());
    }

    public void stop() {
        setLaunchSpeed(0);
        setFeederSpeed(0);
    }

    public void intake() {
        launcher.set(Settings.Launcher.LAUNCHER_INTAKE_SPEED);
        feeder.set(Settings.Launcher.FEEDER_INTAKE_SPEED);
    }

    public void launch(Number feederSpeed, Number launcherSpeed) {
        launcher.set(launcherSpeed.doubleValue());
        feeder.set(feederSpeed.doubleValue());
    }

    //********** GETTERS **********//
    public double getLauncherVelocity() {
        return launcher.getEncoder().getVelocity();
    }

    public double getFeederVelocity() {
        return feeder.getEncoder().getVelocity();
    }

    public double getLauncherSpeed() {
        return launcher.getAppliedOutput();
    }

    public double getFeederSpeed() {
        return feeder.getAppliedOutput();
    }

    @Override 
    public void periodic() {
        SmartDashboard.putNumber("Launcher/Launcher Velocity", getLauncherVelocity());
        SmartDashboard.putNumber("Launcher/Feeder Velocity", getFeederVelocity());

        SmartDashboard.putNumber("Launcher/Launcher Output Speed", getLauncherSpeed());
        SmartDashboard.putNumber("Launcher/Feeder Output Speed", getFeederSpeed());

    }
}