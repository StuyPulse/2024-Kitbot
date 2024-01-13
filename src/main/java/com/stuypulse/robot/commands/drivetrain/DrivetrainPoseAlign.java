package com.stuypulse.robot.commands.drivetrain;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.subsystems.drivetrain.AbstractDrivetrain;
import com.stuypulse.robot.subsystems.odometry.AbstractOdometry;
import com.stuypulse.robot.subsystems.vision.AbstractVision;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;
import com.stuypulse.stuylib.streams.numbers.filters.IFilter;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;

public class DrivetrainPoseAlign extends Command {
    private final AbstractDrivetrain drivetrain;
    private final AbstractOdometry odometry;
    private final AbstractVision vision;

    private IFilter speedAdjFilter;

    protected final AnglePIDController angleController;
    protected final PIDController distanceController;

    private final BStream finished;

    public DrivetrainPoseAlign() {
        this.drivetrain = AbstractDrivetrain.getInstance();
        this.odometry = AbstractOdometry.getInstance();
        this.vision = AbstractVision.getInstance();

        this.angleController = new AnglePIDController(Alignment.Rotation.P, Alignment.Rotation.I, Alignment.Rotation.D);
        this.distanceController = new PIDController(Alignment.Translation.P, Alignment.Translation.I, Alignment.Translation.D);

        this.speedAdjFilter = new LowPassFilter(Alignment.SPEED_ADJ_FILTER);

        this.finished = BStream.create(() -> isAligned())
            .filtered(new BDebounceRC.Rising(Alignment.DEBOUNCE_TIME));

        addRequirements(drivetrain, odometry, vision);
    }
    
    public boolean isAligned() {
        return 
            Math.abs(getTurnError().toDegrees()) < Alignment.ALIGNED_THRESHOLD_ANGLE.get() 
            && Math.abs(getDistanceError()) < Alignment.DISTANCE_THRESHOLD.get();
    }

    public Angle getTurnError() {
        Angle cameraAngle = Angle.fromRotation2d(vision.getOutput().get(0).getPrimaryTag().getPose().toPose2d().getTranslation().getAngle());
        Angle robotAngle = Angle.fromRotation2d(odometry.getPose().getRotation());
        return cameraAngle.sub(robotAngle);

    }

    private double updatedTurn() {
        return angleController.update(
            getTurnError(),
            Angle.fromRotation2d(odometry.getPose().getRotation())
        );
    }

    //the distance error to be the distance between the robot and the april tag target
    public double getDistanceError() {
        double cameraDistanceFromTag = vision.getOutput().get(0).getPrimaryTag().getPose().toPose2d().getTranslation().getNorm();
        Translation3d cameraOffsetFromOdometry = new Translation3d(
            Settings.Vision.CAMERA_OFFSET_X.get(),
            Settings.Vision.CAMERA_OFFSET_Y.get(),
            Settings.Vision.CAMERA_OFFSET_Z.get()
        );
        double robotDistanceFromCamera = odometry.getPose().getTranslation().getNorm() - cameraOffsetFromOdometry.getNorm();
        return cameraDistanceFromTag - robotDistanceFromCamera;
    }

    private double updatedSpeed() {
        return distanceController.update(
            getDistanceError(),
            odometry.getPose().getTranslation().getNorm()
        );
    }

    @Override
    public void initialize() {
        odometry.resetOdometery(odometry.getPose());
        angleController.reset();
        distanceController.reset();
    }

    @Override
    public void execute() {  
        drivetrain.arcadeDrive(updatedSpeed(), updatedTurn());
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