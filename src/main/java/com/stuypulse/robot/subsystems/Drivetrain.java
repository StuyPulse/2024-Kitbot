package com.stuypulse.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase{
    private static Drivetrain instance = new Drivetrain();

    public static Drivetrain getInstance() {
        return instance;
    }

    private DifferentialDrive drivetrain;

    CANSparkMax leftFront;
    CANSparkMax leftBack;
    CANSparkMax rightFront;
    CANSparkMax rightBack;

    public Drivetrain() {
        leftFront = new CANSparkMax(Ports.Drivetrain.LEFTFRONT, MotorType.kBrushed);
        leftBack = new CANSparkMax(Ports.Drivetrain.LEFTFRONT, MotorType.kBrushed);
        rightFront = new CANSparkMax(Ports.Drivetrain.LEFTFRONT, MotorType.kBrushed);
        rightBack = new CANSparkMax(Ports.Drivetrain.LEFTFRONT, MotorType.kBrushed);
   
        leftBack.follow(leftFront);
        rightBack.follow(rightFront);
        
        leftFront.setInverted(true);
        rightFront.setInverted(false);

        leftFront.setSmartCurrentLimit(Settings.Drivetrain.kCurrentLimit);
        leftBack.setSmartCurrentLimit(Settings.Drivetrain.kCurrentLimit);
        rightFront.setSmartCurrentLimit(Settings.Drivetrain.kCurrentLimit);
        leftBack.setSmartCurrentLimit(Settings.Drivetrain.kCurrentLimit);
        
        drivetrain = new DifferentialDrive(leftFront, rightFront);
    }

    //********** GETTERS **********
    public double getLeftSpeed() {
        return leftFront.getEncoder().getVelocity();
    }

    public double getRightSpeed() {
        return rightFront.getEncoder().getVelocity();
    }

    public double getLeftVoltage() {
        return leftFront.getAppliedOutput();
    }

    public double getRightVoltage() {
        return rightFront.getAppliedOutput();
    }

    //********** Drive Methods **********
    public void tankDrive(double leftSpeed, double rightSpeed) {
        drivetrain.tankDrive(leftSpeed, rightSpeed);
    }

    public void arcadeDrive(double speed, double rotation) {
        drivetrain.arcadeDrive(speed, rotation);
    }

    public void curvatureDrive(double speed, double rotation, boolean isQuickTurn) {
        drivetrain.curvatureDrive(speed, rotation, isQuickTurn);
    }

     public void stop() {
        drivetrain.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Drivetrain/ Left Speed", getLeftSpeed());
        SmartDashboard.putNumber("Drivetrain/ Right Speed", getRightSpeed());
        SmartDashboard.putNumber("Drivetrain/ Left Voltage", getLeftVoltage());
        SmartDashboard.putNumber("Drivetrain/ Right Voltage", getRightVoltage());

    }

   
}