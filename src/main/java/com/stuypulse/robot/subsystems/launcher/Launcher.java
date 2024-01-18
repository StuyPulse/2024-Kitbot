package com.stuypulse.robot.subsystems.launcher;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase{

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

    public void intake() {
        launcher.set(Settings.Launcher.INTAKE_LAUNCHER_VOLTAGE);
        feeder.set(Settings.Launcher.INTAKE_FEEDER_VOLTAGE);
    }

    public void launch() {
        launcher.set(Settings.Launcher.LAUNCH_LAUNCHER_VOLTAGE);
        feeder.set(Settings.Launcher.LAUNCH_FEEDER_VOLTAGE);
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
        SmartDashboard.putNumber("Launcher/Launcher Speed", getLauncherSpeed());
        SmartDashboard.putNumber("Launcher/Feeder Speed", getFeederSpeed());

        SmartDashboard.putNumber("Launcher/Launcher Voltage", getLauncherVoltage());
        SmartDashboard.putNumber("Launcher/Feeder Voltage", getFeederVoltage());

    }
}