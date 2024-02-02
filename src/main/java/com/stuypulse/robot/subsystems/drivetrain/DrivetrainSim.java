package com.stuypulse.robot.subsystems.drivetrain;
import com.stuypulse.robot.constants.Settings.Drivetrain;
import com.stuypulse.robot.constants.Settings.Drivetrain.*;
import com.stuypulse.stuylib.math.Angle;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DrivetrainSim extends AbstractDrivetrain {

    private final DifferentialDrivetrainSim sim;
    private final Field2d field;

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

        this.field = new Field2d();
        SmartDashboard.putData("Sim Field", field);
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

    public Angle getAngle() {
        return Angle.fromDegrees(sim.getHeading().getDegrees());
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        sim.setInputs(leftVolts * RoboRioSim.getVInVoltage(), rightVolts * RoboRioSim.getVInVoltage());
    }

    public void chassisSpeedsDrive(ChassisSpeeds speeds) {
        //XXX: PLACEHOLDER
        sim.setInputs(
            speeds.vxMetersPerSecond * RoboRioSim.getVInVoltage(), 
            speeds.omegaRadiansPerSecond * Drivetrain.TRACK_WIDTH * RoboRioSim.getVInVoltage()
        );
    }

    public void arcadeDrive(double speed, double angle) {
        double leftVolts = speed + angle;
        double rightVolts = speed - angle;
        tankDriveVolts(leftVolts, rightVolts);
    }

    public void stop() {
        tankDriveVolts(0, 0);
    }
    
    public void periodicChild() {
        sim.update(0.02);
        field.setRobotPose(sim.getPose());

        SmartDashboard.putNumber("SimDrivetrain/Left Distance", getLeftDistance());
        SmartDashboard.putNumber("SimDrivetrain/Right Distance", getRightDistance());
        SmartDashboard.putNumber("SimDrivetrain/Distance", getDistance());
        
    }

    @Override
    public void setCoast() {}

    @Override
    public void setBrake() {}

}
