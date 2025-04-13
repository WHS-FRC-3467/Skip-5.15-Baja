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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Getter;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map.Entry;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import static frc.robot.subsystems.Vision.ObjectVision.ObjectVisionConstants.*;

public class ObjectVision {
    private final ObjectVisionIO[] io;
    private final ObjectVisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;
    private final Supplier<Pose2d> robot;
    public boolean visionHasTarget = false;
    private boolean seesThisTarget = false;
    private boolean anyCameraConnected = false;

    public Trigger anyCamerasConnected = new Trigger(() -> anyCameraConnected);

    @Getter
    private HashMap<ObjectType, List<Pose2d>> objectPoses = new HashMap<>();

    public ObjectVision(Supplier<Pose2d> robotPoseSupplier, ObjectVisionIO... io)
    {
        this.io = io;
        this.robot = robotPoseSupplier;

        // Initialize inputs
        this.inputs = new ObjectVisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new ObjectVisionIOInputsAutoLogged();
        }

        // Initialize disconnected alerts
        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < inputs.length; i++) {
            disconnectedAlerts[i] =
                new Alert(
                    "Vision camera " + Integer.toString(i) + " is disconnected.",
                    AlertType.kWarning);
        }
    }

    public void periodic()
    {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
        }

        // Initialize logging values
        HashMap<ObjectType, List<Pose2d>> allObjectPoses = new HashMap<>();
        HashMap<ObjectType, List<Pose2d>> allObjectPosesAccepted = new HashMap<>();
        HashMap<ObjectType, List<Pose2d>> allObjectPosesRejected = new HashMap<>();

        boolean anyCameraConnected = false;

        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            // Update disconnected alert
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);
            anyCameraConnected = anyCameraConnected || inputs[cameraIndex].connected;

            // Initialize logging values
            HashMap<ObjectType, List<Pose2d>> objectPoses = new HashMap<>();
            HashMap<ObjectType, List<Pose2d>> objectPosesAccepted = new HashMap<>();
            HashMap<ObjectType, List<Pose2d>> objectPosesRejected = new HashMap<>();

            // Loop over object observations
            for (var observation : inputs[cameraIndex].objectObservations) {
                seesThisTarget = true;

                var objectType = observation.object();

                var transform3d = observation.robotToObject();
                var transform2d = new Transform2d(transform3d.getTranslation().toTranslation2d(),
                    transform3d.getRotation().toRotation2d());

                var objectPose = robot.get().transformBy(transform2d);

                // Check whether to reject pose
                boolean rejectPose =
                    observation.confidence() < leastConfidence // Cannot be low confidence
                        || transform2d.getX() > maxXDistance // Cannot be too far away
                        || transform2d.getY() > maxYDistance; // Cannot be too far away

                // Add pose to log
                if (!objectPoses.containsKey(objectType)) {
                    objectPoses.put(objectType, new LinkedList<>());
                    objectPosesAccepted.put(objectType, new LinkedList<>());
                    objectPosesRejected.put(objectType, new LinkedList<>());

                    allObjectPoses.put(objectType, new LinkedList<>());
                    allObjectPosesAccepted.put(objectType, new LinkedList<>());
                    allObjectPosesRejected.put(objectType, new LinkedList<>());
                }

                objectPoses.get(objectType).add(objectPose);
                if (rejectPose) {
                    objectPosesAccepted.get(objectType).add(objectPose);
                } else {
                    objectPosesRejected.get(objectType).add(objectPose);
                }
            }

            // Report to visionHasTarget whether or not vision sees at least one object
            if (seesThisTarget) {
                visionHasTarget = true;
                // Now reset seesThisTarget for next periodic loop
                seesThisTarget = false;
            } else {
                visionHasTarget = false;
            }

            // Log camera data
            for (Entry<ObjectType, List<Pose2d>> entry : objectPoses.entrySet()) {
                var objectType = entry.getKey();

                Logger.recordOutput(
                    "Vision/Object/Camera" + Integer.toString(cameraIndex) + "/ObjectPoses/"
                        + objectType,
                    entry.getValue().toArray(new Pose3d[objectPoses.size()]));
                Logger.recordOutput(
                    "Vision/Object/Camera" + Integer.toString(cameraIndex) + "/ObjectPosesAccepted/"
                        + objectType,
                    objectPosesAccepted.get(objectType)
                        .toArray(new Pose3d[objectPosesAccepted.size()]));
                Logger.recordOutput(
                    "Vision/Object/Camera" + Integer.toString(cameraIndex) + "/ObjectPosesRejected/"
                        + objectType,
                    objectPosesRejected.get(objectType)
                        .toArray(new Pose3d[objectPosesRejected.size()]));

                allObjectPoses.get(objectType).addAll(entry.getValue());
                allObjectPosesAccepted.get(objectType).addAll(objectPosesAccepted.get(objectType));
                allObjectPosesRejected.get(objectType).addAll(objectPosesAccepted.get(objectType));
            }
        }

        this.anyCameraConnected = anyCameraConnected;

        this.objectPoses = allObjectPoses;

        // Log summary data
        for (Entry<ObjectType, List<Pose2d>> entry : allObjectPoses.entrySet()) {
            var objectType = entry.getKey();

            Logger.recordOutput(
                "Vision/Object/Summary/Camera/ObjectPoses/"
                    + objectType,
                entry.getValue().toArray(new Pose3d[objectPoses.size()]));
            Logger.recordOutput(
                "Vision/Object/Summary/Camera/ObjectPosesAccepted/"
                    + objectType,
                allObjectPosesAccepted.get(objectType)
                    .toArray(new Pose3d[allObjectPosesAccepted.size()]));
            Logger.recordOutput(
                "Vision/Object/Summary/Camera/ObjectPosesRejected/"
                    + objectType,
                allObjectPosesRejected.get(objectType)
                    .toArray(new Pose3d[allObjectPosesRejected.size()]));
        }
    }
}
