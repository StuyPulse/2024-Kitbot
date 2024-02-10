package com.stuypulse.robot.commands.drivetrain;

import com.stuypulse.robot.constants.Settings.Drivetrain.Motion;
import com.stuypulse.robot.subsystems.drivetrain.AbstractDrivetrain;
import com.stuypulse.robot.subsystems.odometry.AbstractOdometry;
import com.stuypulse.robot.util.TrajectoryLoader;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class DrivetrainRamsete extends RamseteCommand {
    
    protected boolean resetPosition;
    protected Trajectory trajectory;
    protected AbstractDrivetrain drivetrain;
    protected AbstractOdometry odometry;

    public DrivetrainRamsete(AbstractDrivetrain drivetrain, AbstractOdometry odometry, Trajectory trajectory) {
        super(
            trajectory, 
            odometry::getPose, 
            new RamseteController(), 
            Motion.MOTOR_FEED_FORWARD,
            Motion.KINEMATICS, 
            drivetrain::getWheelSpeeds,
            new PIDController(Motion.PID.kP, Motion.PID.kI, Motion.PID.kD),
            new PIDController(Motion.PID.kP, Motion.PID.kI, Motion.PID.kD),
            drivetrain::tankDriveVolts, 
            drivetrain
        );

        this.resetPosition = true;
        this.trajectory = trajectory;
        this.drivetrain = drivetrain;

        addRequirements(drivetrain, odometry);
    }

    public DrivetrainRamsete(AbstractDrivetrain drivetrain, AbstractOdometry odometry, String pathString) {
        this(drivetrain, odometry, TrajectoryLoader.getTrajectory(pathString));
    }

    public DrivetrainRamsete(AbstractDrivetrain drivetrain, AbstractOdometry odometry, String... pathStrings ) {
        this(drivetrain, odometry, TrajectoryLoader.getTrajectory(pathStrings));
    }

    // [DEFAULT] Resets the drivetrain to the begining of the trajectory
    public DrivetrainRamsete robotRelative() {
        this.resetPosition = true;
        return this;
    }

    // Make the trajectory relative to the field
    public DrivetrainRamsete fieldRelative() {
        this.resetPosition = false;
        return this;
    }

    @Override
    public void initialize() {
        super.initialize();

        if (resetPosition) {
            this.odometry.resetOdometery(trajectory.getInitialPose());
        }
    }
}
