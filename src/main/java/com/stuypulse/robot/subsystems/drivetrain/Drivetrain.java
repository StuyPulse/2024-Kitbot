package com.stuypulse.robot.subsystems.drivetrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

    private static final Drivetrain instance;

    static {
        instance = new Drivetrain();
    }

    public static Drivetrain getInstance() {
        return instance;
    }

    private final DifferentialDrive drivetrain;

    private final CANSparkMax leftFront;
    private final CANSparkMax leftBack;
    private final CANSparkMax rightFront;
    private final CANSparkMax rightBack;

    public Drivetrain() {
        leftFront = new CANSparkMax(Ports.Drivetrain.LEFTFRONT, MotorType.kBrushed);
        leftBack = new CANSparkMax(Ports.Drivetrain.LEFTREAR, MotorType.kBrushed);
        rightFront = new CANSparkMax(Ports.Drivetrain.RIGHTFRONT, MotorType.kBrushed);
        rightBack = new CANSparkMax(Ports.Drivetrain.RIGHTREAR, MotorType.kBrushed);

        Motors.Drivetrain.LEFT.configure(leftFront);
        Motors.Drivetrain.LEFT.configure(leftBack);

        Motors.Drivetrain.RIGHT.configure(rightFront);
        Motors.Drivetrain.RIGHT.configure(rightBack);
   
        leftBack.follow(leftFront);
        rightBack.follow(rightFront);
        
        drivetrain = new DifferentialDrive(leftFront, rightFront);
    }

    //********** GETTERS **********//
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

    public double getLeftDistance() {
        return leftFront.getEncoder().getPosition();
    }

    public double getRightDistance() {
        return rightFront.getEncoder().getPosition();
    }

    //********** Drive Methods **********//
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
        SmartDashboard.putNumber("Drivetrain/Left Speed", getLeftSpeed());
        SmartDashboard.putNumber("Drivetrain/Right Speed", getRightSpeed());
        SmartDashboard.putNumber("Drivetrain/Left Voltage", getLeftVoltage());
        SmartDashboard.putNumber("Drivetrain/Right Voltage", getRightVoltage());
    }
}