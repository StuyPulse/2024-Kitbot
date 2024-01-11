package com.stuypulse.robot.subsystems.drivetrain;
import com.stuypulse.robot.constants.Settings.Drivetrain;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DrivetrainSim extends AbstractDrivetrain{
    private final DifferentialDrivetrainSim sim;
    private final Field2d field;

    public DrivetrainSim() {
        this.sim = new DifferentialDrivetrainSim(
            DCMotor.getNEO(2),
            Drivetrain.GEARING, 
            Drivetrain.J_KG_METER_SQUARED, 
            Drivetrain.MASS_Kg,
            Drivetrain.WHEEL_RADIUS,
            Drivetrain.TRACK_WIDTH, 
            null
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
    
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        sim.setInputs(leftVolts * RoboRioSim.getVInVoltage(), rightVolts * RoboRioSim.getVInVoltage());
    }

    public void arcadeDrive(double speed, double angle) {
        double leftVolts = speed + angle;
        double rightVolts = speed - angle;
        tankDriveVolts(leftVolts, rightVolts);
    }

    public void curvatureDrive(double speed, double angle, boolean isQuickTurn) {
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
    }

}
