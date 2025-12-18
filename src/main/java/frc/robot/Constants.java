// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public final class Constants {
    public static class CameraConstants{
        public static final String camera1Name = "";
        public static final String camera2Name = "";

        public static final double[] camera1Config = {
            0.0, // forward offset from NavX (meters)
            0.0, // Side offset (meters)
            0.0, // Height offset (meters)
            0.0, // Roll (degrees)
            0.0, // Pitch (degrees)
            0.0 // Yaw (degrees)
        };

        public static final double[] camera2Config = {
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        };
    }

    public static class LocationConstants{
        public static final Pose2d examplePose = new Pose2d(1, 1, new Rotation2d(Units.degreesToRadians(180)));
    }

}
