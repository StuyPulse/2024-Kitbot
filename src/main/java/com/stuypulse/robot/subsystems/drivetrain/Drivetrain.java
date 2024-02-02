package com.stuypulse.robot.subsystems.drivetrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.Angle;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends AbstractDrivetrain {

    private final DifferentialDrive drivetrain;

    private final DifferentialDriveKinematics kinematics;
    private final DifferentialDriveOdometry odometry;

    private final CANSparkMax leftFront;
    private final CANSparkMax leftBack;
    private final CANSparkMax rightFront;
    private final CANSparkMax rightBack;

    public Drivetrain() {
        leftBack = new CANSparkMax(Ports.Drivetrain.LEFTREAR, MotorType.kBrushless);
        leftFront = new CANSparkMax(Ports.Drivetrain.LEFTFRONT, MotorType.kBrushless);
  
        rightFront = new CANSparkMax(Ports.Drivetrain.RIGHTFRONT, MotorType.kBrushless);
        rightBack = new CANSparkMax(Ports.Drivetrain.RIGHTREAR, MotorType.kBrushless);

        leftBack.follow(leftFront);
        rightBack.follow(rightFront);

        Motors.Drivetrain.LEFT.configure(leftFront);
        Motors.Drivetrain.LEFT.configure(leftBack);

        Motors.Drivetrain.RIGHT.configure(rightFront);
        Motors.Drivetrain.RIGHT.configure(rightBack);
   
        drivetrain = new DifferentialDrive(leftFront, rightFront);

        kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(24));
        odometry = new DifferentialDriveOdometry(new Rotation2d(), getLeftDistance(), getRightDistance());

        AutoBuilder.configureRamsete(
            this::getPose,
            this::resetPose,
            this::getCurrentSpeeds,
            this::drive,
            new ReplanningConfig(),
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );
    }

    //********** GETTERS **********//
    public double getLeftSpeed() {
        return leftFront.getEncoder().getVelocity();
    }

    public double getRightSpeed() {
        return rightFront.getEncoder().getVelocity();
    }

    public double getVelocity() {
        return (getLeftSpeed() + getRightSpeed()) / 2.0;
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

    private DifferentialDriveWheelPositions getWheelPositions() {
        return new DifferentialDriveWheelPositions(getLeftDistance(), getRightDistance());
    }
 
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(new Rotation2d(), getWheelPositions(), pose);
    }

    private double getAngularVelocity() {
        return (getLeftSpeed() - getRightSpeed()) / Units.inchesToMeters(26);
    }

    public ChassisSpeeds getCurrentSpeeds() {
        return new ChassisSpeeds(getVelocity(), 0, getAngularVelocity());
    }

     //********** Drive Methods **********//
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftFront.setVoltage(leftVolts);
        rightFront.setVoltage(rightVolts);
        drivetrain.feed();
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        drivetrain.tankDrive(leftSpeed, rightSpeed);
    }

    public void arcadeDrive(double speed, double rotation) {
        drivetrain.arcadeDrive(speed, rotation);
    }

    public void curvatureDrive(double speed, double rotation, boolean isQuickTurn) {
        drivetrain.curvatureDrive(speed, rotation, isQuickTurn);
    }

    public void drive(ChassisSpeeds speeds) {
        drivetrain.arcadeDrive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
    }

    public CANSparkMax[] getMotors() {
        return new CANSparkMax[] {
            rightFront, leftFront, rightBack, leftBack
        };
    }

    @Override
    public void setCoast() {
        for (CANSparkMax motor : getMotors()) {
            motor.setIdleMode(IdleMode.kCoast);
            motor.burnFlash();
        }
    }

    @Override
    public void setBrake() {
        for (CANSparkMax motor : getMotors()) {
            motor.setIdleMode(IdleMode.kBrake);
            motor.burnFlash();
        }
    }

    public void stop() {
        drivetrain.stopMotor();
    }

    @Override
    public void periodicChild() {
        odometry.update(new Rotation2d(), getWheelPositions());

        SmartDashboard.putNumber("Drivetrain/Left Speed", getLeftSpeed());
        SmartDashboard.putNumber("Drivetrain/Right Speed", getRightSpeed());

        SmartDashboard.putNumber("Drivetrain/Left Distance", getLeftDistance());
        SmartDashboard.putNumber("Drivetrain/Right Distance", getRightDistance());

        SmartDashboard.putNumber("Drivetrain/Left Voltage", getLeftVoltage());
        SmartDashboard.putNumber("Drivetrain/Right Voltage", getRightVoltage());

        SmartDashboard.putNumber("Drivetrain/Velocity", getVelocity());
        SmartDashboard.putNumber("Drivetrain/Distance", getDistance());

        SmartDashboard.putNumber("Drivetrain/Angle", getAngle().toDegrees());        
    }
}