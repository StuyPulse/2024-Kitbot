package com.stuypulse.robot.commands.launcher;

import com.stuypulse.robot.subsystems.launcher.Launcher;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * WARNING: DONT USE THIS COMMAND
 * PrepareLaunch sets the launcher to the launch speed, spins just the outside wheel of the launcher to allow it to get up to speed before launching
 */
public class LaunchPrepare extends Command {

    private final Launcher launcher;
    private final Number thresholdRPM;
    private final Number launchSpeed;

    public LaunchPrepare(Number launchSpeed, Number thresholdRPM) {
        this.thresholdRPM = thresholdRPM;
        this.launchSpeed = launchSpeed;
        
        launcher = Launcher.getInstance();
        addRequirements(launcher);
    }
    
    @Override
    public void initialize() {
        launcher.setLaunchSpeed(launchSpeed);
    } 

    @Override
    public boolean isFinished() {
        return Math.abs(launcher.getLauncherVelocity() - thresholdRPM.doubleValue()) < 100;
    } 
}
