/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.stuypulse.robot.commands.drivetrain.DrivetrainDrive;
import com.stuypulse.robot.commands.launcher.LaunchPrepare;
import com.stuypulse.robot.commands.launcher.LauncherHoldSpeed;
import com.stuypulse.robot.commands.launcher.LauncherLaunchSpeaker;
import com.stuypulse.robot.commands.launcher.LauncherIntakeNote;
import com.stuypulse.robot.commands.launcher.LauncherLaunch;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.drivetrain.AbstractDrivetrain;
import com.stuypulse.robot.subsystems.launcher.Launcher;
import com.stuypulse.robot.subsystems.odometry.AbstractOdometry;
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
    public final AbstractOdometry odometry = AbstractOdometry.getInstance();
    //public final AbstractVision vision = AbstractVision.getInstance();

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    // Robot container

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
        configureNamedCommands();
        drivetrain.configureAutoBuilder();
        configureAutons();
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {
        drivetrain.setDefaultCommand(new DrivetrainDrive(driver));
        // launcher.setDefaultCommand(new LauncherHoldSpeed(Settings.Launcher.LAUNCHER_SPEAKER_SPEED));
        launcher.setDefaultCommand(new LauncherHoldSpeed(0));
    }

    /**********************/
    /*** NAMED COMMANDS ***/
    /**********************/

    private void configureNamedCommands() {
        NamedCommands.registerCommand("LauncherLaunch", new LauncherLaunchSpeaker());
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {
        configureDriverBindings();
        configureOperatorBindings();
    }

    private void configureDriverBindings() {
        driver.getRightBumper()
            .whileTrue(new LauncherIntakeNote());
    
        driver.getBottomButton()
            .whileTrue(new LaunchPrepare(Settings.Launcher.LAUNCHER_SPEAKER_SPEED, Settings.Launcher.SPEAKER_THRESHOLD_RPM).andThen(new LauncherLaunchSpeaker()));

        driver.getLeftBumper()
            .whileTrue(new LauncherLaunch(Settings.Launcher.FEEDER_SPEAKER_SPEED, Settings.Launcher.LAUNCHER_SPEAKER_SPEED));
    }

    private void configureOperatorBindings() {}

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Autonomous", autonChooser);

        // autonChooser.addOption("One Note", new OneNote(this));
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
