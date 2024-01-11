/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.drivetrain.DriveDrive;
import com.stuypulse.robot.commands.drivetrain.DrivetrainDrive;
import com.stuypulse.robot.commands.drivetrain.DrivetrainDriveForever;
import com.stuypulse.robot.commands.launcher.LaunchPrepare;
import com.stuypulse.robot.commands.launcher.LauncherIntakeNote;
import com.stuypulse.robot.commands.launcher.LauncherStop;
import com.stuypulse.robot.commands.odometry.OdometryRealign;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.drivetrain.AbstractDrivetrain;
import com.stuypulse.robot.subsystems.launcher.Launcher;
import com.stuypulse.robot.subsystems.odometry.AbstractOdometry;
import com.stuypulse.robot.subsystems.vision.AbstractVision;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);
    
    // Subsystem
    public final AbstractDrivetrain drivetrain = AbstractDrivetrain.getInstance();
    public final Launcher launcher = Launcher.getInstance();
    //public final AbstractOdometry odometry = AbstractOdometry.getInstance();
    //public final AbstractVision vision = AbstractVision.getInstance();

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    // Robot container

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {
        drivetrain.setDefaultCommand(new DrivetrainDrive(driver));
        //drivetrain.setDefaultCommand(new DriveDrive(drivetrain, driver));
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {
        configureDriverBindings();
        configureOperatorBindings();
    }

    private void configureDriverBindings() {
        operator.getLeftBumper()
            .onTrue(new LauncherIntakeNote())
            .onFalse(new LauncherStop());
    
        operator.getRightBumper()
            .onTrue(new LaunchPrepare())
            .whileTrue(new LauncherIntakeNote())
            .onFalse(new LauncherStop());
    }

    private void configureOperatorBindings() {
        // odometry
        //driver.getDPadUp().onTrue(new OdometryRealign(Rotation2d.fromDegrees(180)));
        //driver.getDPadLeft().onTrue(new OdometryRealign(Rotation2d.fromDegrees(-90)));
        //driver.getDPadDown().onTrue(new OdometryRealign(Rotation2d.fromDegrees(0)));
        //driver.getDPadRight().onTrue(new OdometryRealign(Rotation2d.fromDegrees(90)));

        driver.getLeftButton().onTrue(new DrivetrainDriveForever(2));
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
