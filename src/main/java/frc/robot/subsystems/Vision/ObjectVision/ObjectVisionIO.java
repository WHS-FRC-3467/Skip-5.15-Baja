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

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.Vision.ObjectVision.ObjectVisionConstants.ObjectType;
import org.littletonrobotics.junction.AutoLog;

public interface ObjectVisionIO {
    @AutoLog
    public static class ObjectVisionIOInputs {
        public boolean connected = false;
        public ObjectObservation[] objectObservations = new ObjectObservation[0];
    }

    /** Represents an object observation sample used for pose estimation. */
    public static record ObjectObservation(
        double timestamp, Transform3d robotToObject, ObjectType object, double confidence) {
    }

    public default void updateInputs(ObjectVisionIOInputs inputs)
    {}
}
