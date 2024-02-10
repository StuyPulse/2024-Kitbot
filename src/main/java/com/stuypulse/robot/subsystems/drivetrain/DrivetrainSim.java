package com.stuypulse.robot.subsystems.drivetrain;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.stuypulse.robot.constants.Settings.Drivetrain;
import com.stuypulse.robot.constants.Settings.Drivetrain.*;
import com.stuypulse.robot.subsystems.odometry.AbstractOdometry;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DrivetrainSim extends AbstractDrivetrain {

    private final DifferentialDrivetrainSim sim;

    public DrivetrainSim() {
        // this.sim = new DifferentialDrivetrainSim(
        //     DCMotor.getNEO(2),
        //     Drivetrain.GEARING, 
        //     Drivetrain.J_KG_METER_SQUARED, 
        //     Drivetrain.MASS_Kg,
        //     Drivetrain.WHEEL_RADIUS,
        //     Drivetrain.TRACK_WIDTH, 
        //     VecBuilder.fill(0, 0, 0, 0, 0, 0, 0)
        // );
        sim = new DifferentialDrivetrainSim(
            LinearSystemId.identifyDrivetrainSystem(
                Feedforward.kV,                        // linear velocity gain
                Feedforward.kA,                        // linear acceleration gain
                Feedforward.kVAngular,                 // angular velocity gain
                Feedforward.kAAngular,                 // angular acceleration gain
                Drivetrain.TRACK_WIDTH                            // track width of the drivetrain
            ),
            DCMotor.getNEO(6), 
            Drivetrain.GEARING,                 
            Drivetrain.TRACK_WIDTH,                
            Drivetrain.WHEEL_RADIUS,               

            // give the drivetrain measurement noise (none in this example)
            VecBuilder.fill(0, 0, 0, 0, 0, 0, 0)
		);
    }

    public double getLeftDistance() {
        return sim.getLeftPositionMeters();
    }

    public double getRightDistance() {
        return sim.getRightPositionMeters();
    }

    public double getDistance() {
        return (getLeftDistance() + getRightDistance()) / 2.0;
    }

    public double getLeftVelocity() {
        return sim.getLeftVelocityMetersPerSecond();
    }

    public double getRightVelocity() {
        return sim.getRightVelocityMetersPerSecond();
    }

    public double getVelocity() {
        return (getLeftVelocity() + getRightVelocity()) / 2.0;
    }

    public Rotation2d getGyroAngle() {
        return sim.getHeading();
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        sim.setInputs(leftVolts * RoboRioSim.getVInVoltage(), rightVolts * RoboRioSim.getVInVoltage());
    }

    public void chassisSpeedsDrive(ChassisSpeeds speeds) {}

    public void arcadeDrive(double speed, double angle) {
        double leftVolts = speed + angle;
        double rightVolts = speed - angle;
        tankDriveVolts(leftVolts, rightVolts);
    }

    public void stop() {
        tankDriveVolts(0, 0);
    }
    
    @Override
    public void simulationPeriodic() {
        sim.update(0.02);
        AbstractOdometry.getInstance().getField().setRobotPose(sim.getPose());
        
        SmartDashboard.putNumber("SimDrivetrain/Left Distance", getLeftDistance());
        SmartDashboard.putNumber("SimDrivetrain/Right Distance", getRightDistance());
        SmartDashboard.putNumber("SimDrivetrain/Distance", getDistance());
    }

    @Override
    public void setCoast() {}

    @Override
    public void setBrake() {}

    @Override
    public void curvatureDrive(double speed, double rotation, boolean isQuickTurn) {}

    public void configureAutoBuilder() {
        AbstractOdometry odometry = AbstractOdometry.getInstance();

        AutoBuilder.configureRamsete(
            odometry::getPose, // Robot pose supplier
            odometry::resetOdometery, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // Current ChassisSpeeds supplier
            (ChassisSpeeds speeds) -> { // Method that will drive the robot given ChassisSpeeds
                getInstance().arcadeDrive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
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
