package com.stuypulse.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase{
    private static Launcher instance = new Launcher();

    public static Launcher getInstance() {
        return instance;
    }

    CANSparkMax feeder;
    CANSparkMax launcher;
    
    public Launcher() {
        feeder = new CANSparkMax(Ports.Launcher.FEEDER, MotorType.kBrushed); 
        launcher = new CANSparkMax(Ports.Launcher.LAUNCHER, MotorType.kBrushed);

        feeder.setSmartCurrentLimit(Settings.Launcher.kFeedCurrentLimit);
        launcher.setSmartCurrentLimit(Settings.Launcher.kLauncherCurrentLimit);
    }

    //********** SETTERS **********
    public void setLaunchSpeed(double speed) {
        launcher.set(speed);
    }

    public void setFeederSpeed(double speed) {
        feeder.set(speed);
    }

    public void stop() {
        setLaunchSpeed(0);
        setFeederSpeed(0);
    }

    public void launch() {
        launcher.set(Settings.Launcher.kLauncherSpeed);
        feeder.set(Settings.Launcher.kLaunchFeederSpeed);
    }

    public void intake() {
        launcher.set(Settings.Launcher.kIntakeLauncherSpeed);
        feeder.set(Settings.Launcher.kIntakeFeederSpeed);
    }

    //********** GETTERS **********
    public double getLauncherSpeed() {
        return launcher.getEncoder().getVelocity();
    }

    public double getFeederSpeed() {
        return feeder.getEncoder().getVelocity();
    }

    public double getLauncherVoltage() {
        return launcher.getAppliedOutput();
    }

    public double getFeederVoltage() {
        return feeder.getAppliedOutput();
    }

    @Override 
    public void periodic() {
        SmartDashboard.putNumber("Launcher/ Launcher Speed", getLauncherSpeed());
        SmartDashboard.putNumber("Launcher/ Feeder Speed", getFeederSpeed());
        SmartDashboard.putNumber("Launcher/ Launcher Voltage", getLauncherVoltage());
        SmartDashboard.putNumber("Launcher/ Feeder Voltage", getFeederVoltage());

    }
}