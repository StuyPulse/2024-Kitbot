package com.stuypulse.robot.commands;

import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.subsystems.drivetrain.AbstractDrivetrain;
import com.stuypulse.robot.subsystems.odometry.AbstractOdometry;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.vision.AbstractVision;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.angles.AFuser;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;
import com.stuypulse.stuylib.streams.numbers.filters.IFilter;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;

import edu.wpi.first.wpilibj2.command.Command;

public class AngleAlign extends Command {
    private final AbstractDrivetrain drivetrain;
    private final AbstractOdometry odometry;
    private final AbstractVision vision;

    private final AFuser angleError;
    private IFilter speedAdjFilter;

    protected final AnglePIDController angleController;

    private final BStream finished;

    public AngleAlign() {
        this.drivetrain = AbstractDrivetrain.getInstance();
        this.odometry = Odometry.getInstance();
        this.vision = AbstractVision.getInstance();

        this.angleError = new AFuser(Alignment.FUSION_FILTER.get(),
            () -> Angle.fromRotation2d(vision.getOutput().get(0).getPrimaryTag().getPose().toPose2d().getRotation()),
            () -> Angle.fromRotation2d(odometry.getPose().getRotation())
        );

        this.speedAdjFilter = new LowPassFilter(Alignment.SPEED_ADJ_FILTER.get());

        this.angleController = new AnglePIDController(Alignment.Rotation.P, Alignment.Rotation.I, Alignment.Rotation.D);
        
        this.finished = BStream.create(this::isAligned)
            //TODO: check to see if it is degrees or radians
            .and(() -> angleController.isDoneDegrees(Alignment.ALIGNED_THRESHOLD_ANGLE.get()))
            .filtered(new BDebounceRC.Rising(Alignment.DEBOUNCE_TIME));

        addRequirements(drivetrain, odometry, vision);
    }

    public boolean isAligned() {
        // check to see if the robot is within a threshold of the april tag target
        return Math.abs(angleError.get().toDegrees()) < Alignment.ALIGNED_THRESHOLD_ANGLE.get();
    }

    private double getTurn() {
        // //calcs with the setpoint angle of line between tag and robot to the current angle of the robot
        // return angleController.update(
        //     /* angle of line between tag and robot*/
        //     angleController.getSetpoint()
        //     , 
        //     odometry.getRotation());
        return 0;
    }

    @Override
    public void initialize() {
        odometry.resetOdometery(odometry.getPose());
        angleController.reset();
        angleError.reset();
    }

    @Override
    public void execute() {
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
