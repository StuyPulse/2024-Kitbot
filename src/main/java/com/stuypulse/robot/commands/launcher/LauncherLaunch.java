package com.stuypulse.robot.commands.launcher;

import com.stuypulse.robot.subsystems.launcher.Launcher;

import edu.wpi.first.wpilibj2.command.Command;

public class LauncherLaunch extends Command {

    private final Launcher launcher;
    private final Number feederSpeed;
    private final Number launcherSpeed;

    public LauncherLaunch(Number feederSpeed, Number launcherSpeed) {
        this.launcherSpeed = launcherSpeed;
        this.feederSpeed = feederSpeed;

        launcher = Launcher.getInstance();
        addRequirements(launcher);
    }
    
    @Override
    public void initialize() {
        launcher.launch(feederSpeed, launcherSpeed);
    }
}