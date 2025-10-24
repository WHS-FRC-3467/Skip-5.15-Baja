// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTuneableProfiledPID;

public class AlignToPose extends AlignToPoseBase {

    private final static LoggedTuneableProfiledPID linearController =
        new LoggedTuneableProfiledPID("DriveToPose/LinearController", 3.0, 0, 0.1, 3.0, 0.0);

    private final static LoggedTuneableProfiledPID angularController =
        new LoggedTuneableProfiledPID("DriveToPose/AngularController", 3.0, 0, 0, 0, 0);

    public AlignToPose(Drive drive, Supplier<Pose2d> targetPose, AlignMode mode,
        DoubleSupplier joystickInput)
    {
        super(drive,
            () -> targetPose.get().transformBy(new Transform2d(0, 0, Rotation2d.k180deg)),
            mode,
            joystickInput,
            linearController,
            angularController);
    }
}
