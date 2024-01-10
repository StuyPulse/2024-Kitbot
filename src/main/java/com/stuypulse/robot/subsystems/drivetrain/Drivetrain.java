package com.stuypulse.robot.subsystems.drivetrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.Angle;

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

    RelativeEncoder leftEncoder;
    RelativeEncoder rightEncoder;

    public Drivetrain() {
        leftFront = new CANSparkMax(Ports.Drivetrain.LEFTFRONT, MotorType.kBrushed);
        leftBack = new CANSparkMax(Ports.Drivetrain.LEFTFRONT, MotorType.kBrushed);
        rightFront = new CANSparkMax(Ports.Drivetrain.LEFTFRONT, MotorType.kBrushed);
        rightBack = new CANSparkMax(Ports.Drivetrain.LEFTFRONT, MotorType.kBrushed);
   
        leftBack.follow(leftFront);
        rightBack.follow(rightFront);
        
        leftFront.setInverted(true);
        rightFront.setInverted(false);

        leftFront.setSmartCurrentLimit(Settings.Drivetrain.CURRENT_LIMIT);
        leftBack.setSmartCurrentLimit(Settings.Drivetrain.CURRENT_LIMIT);
        rightFront.setSmartCurrentLimit(Settings.Drivetrain.CURRENT_LIMIT);
        leftBack.setSmartCurrentLimit(Settings.Drivetrain.CURRENT_LIMIT);
        
        leftEncoder = leftFront.getEncoder();
        rightEncoder = rightFront.getEncoder();
        resetEncoders();

        drivetrain = new DifferentialDrive(leftFront, rightFront);
    }

    //********** GETTERS **********
    // Distance
    public double getLeftDistance() {
        return leftEncoder.getPosition();
    }

    public double getRightDistance() {
        return rightEncoder.getPosition();
    }

    public double getDistance() {
        return (getLeftDistance() + getRightDistance()) / 2.0;
    }

    // Velocity
    public double getLeftVelocity() {
        return leftEncoder.getVelocity();
    }

    public double getRightVelocity() {
        return rightEncoder.getVelocity();
    }

    public double getVelocity() {
        return (getLeftVelocity() + getRightVelocity()) / 2.0;
    }

    public double getLeftVoltage() {
        return leftFront.getAppliedOutput();
    }

    public double getRightVoltage() {
        return rightFront.getAppliedOutput();
    }

    private double getRawEncoderAngle() {
        double distance = getLeftDistance() - getRightDistance();
        return Math.toDegrees(distance / Settings.Drivetrain.TRACK_WIDTH);
    }

    public Angle getAngle() {
        return Angle.fromDegrees(getRawEncoderAngle());
    }

    public void resetEncoders() {
        leftEncoder.setPosition(0.0);
        rightEncoder.setPosition(0.0);
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
        SmartDashboard.putNumber("Drivetrain/ Left Velocity (m per s)", getLeftVelocity());
        SmartDashboard.putNumber("Drivetrain/ Right Velocity (m per s)", getRightVelocity());

        SmartDashboard.putNumber("Drivetrain/ Left Distance (m)", getLeftDistance());
        SmartDashboard.putNumber("Drivetrain/ Right Distance (m)", getRightDistance());

        SmartDashboard.putNumber("Drivetrain/ Angle (deg)", getAngle().toDegrees());


        SmartDashboard.putNumber("Drivetrain/ Left Voltage (V)", getLeftVoltage());
        SmartDashboard.putNumber("Drivetrain/ Right Voltage (V)", getRightVoltage());

    }

   
}