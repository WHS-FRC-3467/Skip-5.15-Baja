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
import java.util.LinkedList;
import java.util.List;
import org.photonvision.PhotonCamera;

/** IO implementation for real PhotonVision hardware. */
public class ObjectVisionIOPhotonVision implements ObjectVisionIO {
    protected final PhotonCamera camera;
    protected final Transform3d robotToCamera;
    protected final ObjectType object;

    /**
     * Creates a new VisionIOPhotonVision.
     *
     * @param name The configured name of the camera.
     * @param robotToCamera The 3D position of the camera relative to the robot.
     */
    public ObjectVisionIOPhotonVision(String name, Transform3d robotToCamera, ObjectType object)
    {
        camera = new PhotonCamera(name);
        this.robotToCamera = robotToCamera;
        this.object = object;
    }

    @Override
    public void updateInputs(ObjectVisionIOInputs inputs)
    {
        inputs.connected = camera.isConnected();

        // Read new camera observations
        List<ObjectObservation> observations = new LinkedList<>();
        for (var result : camera.getAllUnreadResults()) {
            if (result.hasTargets()) {
                for (var target : result.targets) {
                    // Calculate object pose
                    Transform3d cameraToTarget = target.bestCameraToTarget;
                    Transform3d robotToTarget = robotToCamera.plus(cameraToTarget);

                    // Add observation
                    observations.add(
                        new ObjectObservation(
                            result.getTimestampSeconds(), // Timestamp
                            robotToTarget, // 3D transform estimate
                            object,
                            target.poseAmbiguity // Ambiguity
                        ));
                }
            }
        }

        // Save pose observations to inputs object
        inputs.objectObservations = new ObjectObservation[observations.size()];
        for (int i = 0; i < observations.size(); i++) {
            inputs.objectObservations[i] = observations.get(i);
        }
    }
}
