package com.stuypulse.robot.commands.launcher;

import com.stuypulse.robot.subsystems.launcher.Launcher;

import edu.wpi.first.wpilibj2.command.Command;

public class LauncherHoldSpeed extends Command {
    
    private final Launcher launcher;
    private final Number launcherSpeed;

    public LauncherHoldSpeed(Number launcherSpeed) {
        launcher = Launcher.getInstance();
        this.launcherSpeed = launcherSpeed;

        addRequirements(launcher);
    }

    @Override
    public void execute() {
        launcher.setFeederSpeed(0);
        launcher.setLaunchSpeed(launcherSpeed);
    }

}
