package com.stuypulse.robot.subsystems.drivetrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.Angle;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends AbstractDrivetrain {
    private final DifferentialDrive drivetrain;

    private final CANSparkMax leftFront;
    private final CANSparkMax leftBack;
    private final CANSparkMax rightFront;
    private final CANSparkMax rightBack;

    public Drivetrain() {
        //TODO: changed to brushless for simulation
        leftFront = new CANSparkMax(Ports.Drivetrain.LEFTFRONT, MotorType.kBrushless);
        leftBack = new CANSparkMax(Ports.Drivetrain.LEFTREAR, MotorType.kBrushless);
        rightFront = new CANSparkMax(Ports.Drivetrain.RIGHTFRONT, MotorType.kBrushless);
        rightBack = new CANSparkMax(Ports.Drivetrain.RIGHTREAR, MotorType.kBrushless);

        Motors.Drivetrain.LEFT.configure(leftFront);
        Motors.Drivetrain.LEFT.configure(leftBack);

        Motors.Drivetrain.RIGHT.configure(rightFront);
        Motors.Drivetrain.RIGHT.configure(rightBack);
   
        leftBack.follow(leftFront);
        rightBack.follow(rightFront);
        
        drivetrain = new DifferentialDrive(leftFront, rightFront);
    }

    //********** GETTERS **********//
    public double getLeftVelocity() {
        return leftFront.getEncoder().getVelocity();
    }

    public double getRightVelocity() {
        return rightFront.getEncoder().getVelocity();
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

    public double getLeftDistance() {
        return leftFront.getEncoder().getPosition();
    }

    public double getRightDistance() {
        return rightFront.getEncoder().getPosition();
    }

    public double getDistance() {
        return (getLeftDistance() + getRightDistance()) / 2.0;
    } 

    public Angle getAngle() {
        return Angle.fromDegrees(Math.toDegrees(getLeftDistance() - getRightDistance() / Settings.Drivetrain.TRACK_WIDTH));
    }
    
    //********** Drive Methods **********//
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftFront.setVoltage(leftVolts);
        rightFront.setVoltage(-rightVolts);
        drivetrain.feed();
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        drivetrain.tankDrive(leftSpeed, rightSpeed);
    }

    public void chassisSpeedsDrive(ChassisSpeeds speeds) {
        arcadeDrive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond * Settings.Drivetrain.TRACK_WIDTH);
      
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
    public void periodicChild() {
        SmartDashboard.putNumber("Drivetrain/Left Speed", getLeftVelocity());
        SmartDashboard.putNumber("Drivetrain/Right Speed", getRightVelocity());
        SmartDashboard.putNumber("Drivetrain/Left Voltage", getLeftVoltage());
        SmartDashboard.putNumber("Drivetrain/Right Voltage", getRightVoltage());
    }
}