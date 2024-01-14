package com.stuypulse.robot.commands.drivetrain;

import com.stuypulse.robot.subsystems.drivetrain.AbstractDrivetrain;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.numbers.IStream;

import edu.wpi.first.wpilibj2.command.Command;

public class DrivetrainDrive extends Command {
    public final AbstractDrivetrain drivetrain;

    double rightSpeed;
    double leftSpeed;

    private final IStream speed;
    private final IStream angle;

    public DrivetrainDrive(Gamepad driver) {
        this.drivetrain = AbstractDrivetrain.getInstance();

        this.speed = IStream.create(() -> driver.getRightTrigger() - driver.getLeftTrigger())
            .filtered(
                x -> SLMath.deadband(x, 0),
                x -> SLMath.spow(x, 2)
        );

        this.angle = IStream.create(() -> driver.getLeftX())
            .filtered(
                x -> SLMath.deadband(x, 0),
                x -> SLMath.spow(x, 2),
                x -> -x
        );

        addRequirements(drivetrain);

    }

    @Override
    public void execute() {
        drivetrain.curvatureDrive(speed.get(), angle.get(), true);
    }
}
