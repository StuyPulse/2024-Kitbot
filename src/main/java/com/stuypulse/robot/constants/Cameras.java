package com.stuypulse.robot.constants;


import com.stuypulse.robot.util.CustomCamera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

/*-
 * File containing all of the configurations that different motors require.
 *
 * Such configurations include:
 *  - If it is Inverted
 *  - The Idle Mode of the Motor
 *  - The Current Limit
 *  - The Open Loop Ramp Rate
 */
public interface Cameras {

    public static final CameraConfig DEFAULT_CAMERA = new CameraConfig("default",
            new Pose3d(Units.inchesToMeters(12.5),
                       Units.inchesToMeters(11.5),
                       Units.inchesToMeters(8.5),
                       new Rotation3d(0, 0, 0)));

    public static final CameraConfig[] ROBOT_CAMERAS = new CameraConfig[]{DEFAULT_CAMERA};

    public static class CameraConfig {
        public final String NAME;
        public final Pose3d POSITION;

        public CameraConfig(String name, Pose3d position) {
            NAME = name;
            POSITION = position;
        }

        public CustomCamera getCamera() {
            return new CustomCamera(NAME, POSITION);
        }
    }
}
