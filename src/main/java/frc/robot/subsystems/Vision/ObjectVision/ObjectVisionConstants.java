// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.Vision.ObjectVision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class ObjectVisionConstants {

    // Possible objects to detect in order of ID
    public enum ObjectType {
        ALGAE // 0
    }

    // Camera name, must match name configured on coprocessor
    public static String cameraName = "algae_cam";

    // Robot to camera transforms
    public static Transform3d robotToCamera =
        new Transform3d(Units.inchesToMeters(9.287), Units.inchesToMeters(10.9704),
            Units.inchesToMeters(7.9167),
            new Rotation3d(0.0, Units.degreesToRadians(-15), Units.degreesToRadians(-30)));

    // Basic filtering thresholds
    public static double leastConfidence = 0.7;
    public static double maxXDistance = 3;
    public static double maxYDistance = 1;
}
