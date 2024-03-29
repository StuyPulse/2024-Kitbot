package com.stuypulse.robot.util;

import edu.wpi.first.math.geometry.Pose3d;

public class Fiducial {
    private final int id;
    private final Pose3d pose;

    public Fiducial(int id, Pose3d pose) {
        this.id = id;
        this.pose = pose;
    }

    public int getID() {
        return id;
    }

    public Pose3d getPose() {
        return pose;
    }
}
