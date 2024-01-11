package com.stuypulse.robot.commands.drivetrain;

import com.stuypulse.robot.subsystems.drivetrain.AbstractDrivetrain;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.wpilibj2.command.Command;

// DriveCommand.java
public class DriveDrive extends Command {

    private AbstractDrivetrain drivetrain;
	private Gamepad gamepad;

	public DriveDrive(AbstractDrivetrain drivetrain, Gamepad gamepad) {
		this.drivetrain = drivetrain;
		this.gamepad = gamepad;

		addRequirements(drivetrain);
	}

	public void execute() {
		drivetrain.tankDriveVolts(gamepad.getLeftTrigger(), gamepad.getRightTrigger());
	}

}