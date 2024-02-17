package com.stuypulse.robot.commands.drivetrain;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.subsystems.drivetrain.AbstractDrivetrain;
import com.stuypulse.robot.subsystems.odometry.AbstractOdometry;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;

import edu.wpi.first.wpilibj2.command.Command;

public class DrivetrainTurn extends Command{
    private final AbstractDrivetrain drivetrain;
    private final AbstractOdometry odometry;

    protected final AnglePIDController angleController;
    private final Angle targetAngle;

    private final BStream finished;

    public DrivetrainTurn(Angle targetAngle) {
        drivetrain = AbstractDrivetrain.getInstance();
        odometry = AbstractOdometry.getInstance();
        this.targetAngle = targetAngle;

        angleController = new AnglePIDController(Alignment.Rotation.P, Alignment.Rotation.I, Alignment.Rotation.D);

        finished = BStream.create(this::isAtTargetAngle)
            .and(() -> angleController.isDoneDegrees(Alignment.ALIGNED_THRESHOLD_ANGLE.get()))
            .filtered(new BDebounceRC.Rising(Alignment.DEBOUNCE_TIME));

        addRequirements(drivetrain);
    }

    public boolean isAtTargetAngle() {
        return Math.abs(getTurnError().toDegrees()) < Alignment.ALIGNED_THRESHOLD_ANGLE.get();
    }

    public Angle getTurnError() {
        Angle robotAngle = Angle.fromRotation2d(odometry.getPose().getRotation());
        return targetAngle.sub(robotAngle);
    }

    private double updatedTurn() {
        return angleController.update(
         getTurnError(),
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
