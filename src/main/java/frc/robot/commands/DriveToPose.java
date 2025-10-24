// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.LoggedTuneableProfiledPID;
import lombok.Getter;

public class DriveToPose extends DriveToPoseBase {

    private final static LoggedTuneableProfiledPID linearController =
        new LoggedTuneableProfiledPID("DriveToPose/LinearController", 3.0, 0, 0.1, 3.0, 0.0);
    private final static LoggedTuneableProfiledPID angularController =
        new LoggedTuneableProfiledPID("DriveToPose/AngularController", 3.0, 0, 0, 0, 0);

    private final static LoggedTunableNumber maxLinearVel =
        new LoggedTunableNumber("DriveToPose/MaxLinearVelocity (m s)", 3.0);
    private final static LoggedTunableNumber maxAngularVel =
        new LoggedTunableNumber("DriveToPose/MaxAngularVelocity (rad s)", 9.0);

    public DriveToPose(
        Drive drive,
        Supplier<Pose2d> targetPose)
    {
        super(
            drive,
            targetPose,
            linearController,
            angularController,
            maxLinearVel,
            maxAngularVel);


    }



}
