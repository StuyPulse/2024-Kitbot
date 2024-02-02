

package com.stuypulse.robot.commands.drivetrain;

import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.subsystems.drivetrain.AbstractDrivetrain;
import com.stuypulse.robot.subsystems.odometry.AbstractOdometry;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.vision.AbstractVision;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;

import edu.wpi.first.wpilibj2.command.Command;

public class DrivetrainAngleAlign extends Command {
    private final AbstractDrivetrain drivetrain;
    private final AbstractOdometry odometry;
    private final AbstractVision vision;

    protected final AnglePIDController angleController;

    private final BStream finished;

    public DrivetrainAngleAlign() {
        this.drivetrain = AbstractDrivetrain.getInstance();
        this.odometry = Odometry.getInstance();
        this.vision = AbstractVision.getInstance();

        this.angleController = new AnglePIDController(Alignment.Rotation.P, Alignment.Rotation.I, Alignment.Rotation.D);
        
        this.finished = BStream.create(this::isAligned)
            //TODO: check to see if it is degrees or radians
            .and(() -> angleController.isDoneDegrees(Alignment.ALIGNED_THRESHOLD_ANGLE.get()))
            .filtered(new BDebounceRC.Rising(Alignment.DEBOUNCE_TIME));

        addRequirements(drivetrain, odometry, vision);
    }
    
    public boolean isAligned() {
        // check to see if the robot is within a threshold of the april tag target 
        //TODO: change the threshold constant to be different from line 34
        return Math.abs(getTurnError().toDegrees()) < Alignment.ALIGNED_THRESHOLD_ANGLE.get();
    }

    public Angle getTurnError() {
        // get the angle between the robot and the april tag target
        Angle cameraAngle = Angle.fromRotation2d(vision.getOutput().get(0).getPrimaryTag().getPose().toPose2d().getTranslation().getAngle());
        Angle robotAngle = Angle.fromRotation2d(odometry.getPose().getRotation());
        return cameraAngle.sub(robotAngle);

    }

    private double updatedTurn() {
           //calcs with the setpoint angle of line between tag and robot to the current angle of the robot
           return angleController.update(
            /* angle between robot and target */
            getTurnError(),
            /* current angle of the robot from odometry pose */
            Angle.fromRotation2d(odometry.getPose().getRotation())
        );
    }

    @Override
    public void initialize() {
        odometry.resetOdometery(odometry.getPose());
        angleController.reset();
    }

    @Override
    public void execute() {  
        drivetrain.arcadeDrive(0, updatedTurn());
    }

    @Override 
    public boolean isFinished() {
        return finished.get();
    }

    @Override 
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
