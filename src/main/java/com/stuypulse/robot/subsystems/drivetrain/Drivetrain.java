package com.stuypulse.robot.subsystems.drivetrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.Angle;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

import com.stuypulse.robot.subsystems.odometry.AbstractOdometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;


public class Drivetrain extends AbstractDrivetrain {

    private final DifferentialDrive drivetrain;

    private final DifferentialDriveKinematics kinematics;
    private final DifferentialDriveOdometry odometry;

    private final CANSparkMax leftFront;
    private final CANSparkMax leftBack;
    private final CANSparkMax rightFront;
    private final CANSparkMax rightBack;

    private final AHRS navX;

    public Drivetrain() {
        leftBack = new CANSparkMax(Ports.Drivetrain.LEFTREAR, MotorType.kBrushless);
        leftFront = new CANSparkMax(Ports.Drivetrain.LEFTFRONT, MotorType.kBrushless);
  
        rightFront = new CANSparkMax(Ports.Drivetrain.RIGHTFRONT, MotorType.kBrushless);
        rightBack = new CANSparkMax(Ports.Drivetrain.RIGHTREAR, MotorType.kBrushless);

        leftBack.getEncoder().setPositionConversionFactor(Settings.Drivetrain.GEARING);
        leftFront.getEncoder().setPositionConversionFactor(Settings.Drivetrain.GEARING);
        rightBack.getEncoder().setPositionConversionFactor(Settings.Drivetrain.GEARING);
        rightFront.getEncoder().setPositionConversionFactor(Settings.Drivetrain.GEARING);
        
        leftBack.follow(leftFront);
        rightBack.follow(rightFront);

        Motors.Drivetrain.LEFT.configure(leftFront);
        Motors.Drivetrain.LEFT.configure(leftBack);

        Motors.Drivetrain.RIGHT.configure(rightFront);
        Motors.Drivetrain.RIGHT.configure(rightBack);
   
        drivetrain = new DifferentialDrive(leftFront, rightFront);

        navX = new AHRS(SPI.Port.kMXP);

        kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(24));
        odometry = new DifferentialDriveOdometry(getGyroAngle(), getLeftDistance(), getRightDistance());
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

    @Override
    public Rotation2d getGyroAngle() {
        // return Rotation2d.fromRadians((getLeftDistance() - getRightDistance()) / Settings.Drivetrain.TRACK_WIDTH);
        return Rotation2d.fromDegrees(navX.getAngle());
    }

    private DifferentialDriveWheelPositions getWheelPositions() {
        return new DifferentialDriveWheelPositions(getLeftDistance(), getRightDistance());
    }
 
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(getGyroAngle(), getWheelPositions(), pose);
    }

    private double getAngularVelocity() {
        return (getLeftVelocity() - getRightVelocity()) / Units.inchesToMeters(26);
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

    public void chassisSpeedsDrive(ChassisSpeeds speeds) {
        arcadeDrive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond / 3.0);
    }

    public void arcadeDrive(double speed, double rotation) {
        drivetrain.arcadeDrive(speed, -rotation);
    }

    public void curvatureDrive(double speed, double rotation, boolean isQuickTurn) {
        drivetrain.curvatureDrive(speed, -rotation, isQuickTurn);
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
        SmartDashboard.putNumber("Drivetrain/Left Speed", getLeftVelocity());
        SmartDashboard.putNumber("Drivetrain/Right Speed", getRightVelocity());
        SmartDashboard.putNumber("Drivetrain/Left Voltage", getLeftVoltage());
        SmartDashboard.putNumber("Drivetrain/Right Voltage", getRightVoltage());

        SmartDashboard.putNumber("Drivetrain/Velocity", getVelocity());
        SmartDashboard.putNumber("Drivetrain/Distance", getDistance());

        SmartDashboard.putNumber("Drivetrain/Angle", getGyroAngle().getDegrees());     
        SmartDashboard.putNumber("Drivetrain/Gyro Angle", getGyroAngle().getDegrees());     
    }

    public void configureAutoBuilder() {
        AbstractOdometry odometry = AbstractOdometry.getInstance();

        AutoBuilder.configureRamsete(
            odometry::getPose, // Robot pose supplier
            odometry::resetOdometery, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // Current ChassisSpeeds supplier
            (ChassisSpeeds speeds) -> { // Method that will drive the robot given ChassisSpeeds
                getInstance().arcadeDrive(speeds.vxMetersPerSecond / 3, speeds.omegaRadiansPerSecond / 2 / Math.PI);
            },
            new ReplanningConfig(), // Default path replanning config. See the API for the options here
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this // Reference to this subsystem to set requirements
        );
    }
}