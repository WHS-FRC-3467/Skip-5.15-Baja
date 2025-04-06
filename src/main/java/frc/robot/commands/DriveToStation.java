// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Reef;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import frc.robot.util.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

public class DriveToStation extends DriveToPose {
    private static final LoggedTunableNumber stationAlignDistance =
        new LoggedTunableNumber(
            "DriveToStation/StationAlignDistanceInches", 1);
    private static final LoggedTunableNumber horizontalMaxOffset =
        new LoggedTunableNumber(
            "DriveToStation/HorizontalMaxOffsetInches", 25);

    public DriveToStation(Drive drive)
    {
        this(drive, () -> 0, () -> 0, () -> 0);
    }

    public DriveToStation(
        Drive drive,
        DoubleSupplier driverX,
        DoubleSupplier driverY,
        DoubleSupplier driverOmega)
    {
        this(
            drive,
            () -> DriveCommands.getLinearVelocityFromJoysticks(
                driverX.getAsDouble(), driverY.getAsDouble())
                .times(AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0),
            () -> Math.copySign(
                Math.pow(
                    MathUtil.applyDeadband(driverOmega.getAsDouble(), 0.1), 2.0),
                driverOmega.getAsDouble()));
    }

    public DriveToStation(
        Drive drive, Supplier<Translation2d> linearOverride, DoubleSupplier theta)
    {
        super(
            drive,
            () -> {
                Pose2d robot = AllianceFlipUtil.apply(drive.getPose());
                List<Pose2d> finalPoses = new ArrayList<>();
                for (Pose2d stationCenter : new Pose2d[] {
                        FieldConstants.CoralStation.leftCenterFace,
                        FieldConstants.CoralStation.rightCenterFace
                }) {
                    Transform2d offset = new Transform2d(stationCenter, robot);
                    offset =
                        new Transform2d(
                            Constants.bumperWidth / 2.0
                                + Units.inchesToMeters(stationAlignDistance.get()),
                            MathUtil.clamp(
                                offset.getY(), -Units.inchesToMeters(horizontalMaxOffset.get()),
                                Units.inchesToMeters(horizontalMaxOffset.get())),
                            Rotation2d.kZero);

                    finalPoses.add(stationCenter.transformBy(offset));
                }
                Pose2d intakePose = AllianceFlipUtil.apply(robot.nearest(finalPoses));
                robot = AllianceFlipUtil.apply(robot);
                Pose2d goal;
                if (AllianceFlipUtil.apply(robot).getTranslation()
                    .getDistance(FieldConstants.Reef.centerOfReef) <= Reef.faceLength
                        + Constants.bumperWidth / 2.0
                        + 0.35) {
                    final double yError = intakePose.relativeTo(robot).getY();
                    goal =
                        robot
                            .transformBy(
                                new Transform2d(
                                    -4.0, Math.abs(yError) > 0.6 ? yError * 0.6 : yError,
                                    Rotation2d.kZero))
                            .transformBy(
                                new Transform2d(
                                    Translation2d.kZero,
                                    robot
                                        .getRotation()
                                        .interpolate(intakePose.getRotation(), 0.05)
                                        .minus(robot.getRotation())));
                } else {
                    goal = intakePose;
                }

                Logger.recordOutput(
                    "DriveToStation/LeftClosestPose", AllianceFlipUtil.apply(finalPoses.get(0)));
                Logger.recordOutput(
                    "DriveToStation/RightClosestPose", AllianceFlipUtil.apply(finalPoses.get(1)));
                return goal;
            },
            drive::getPose);
        super.withOverride(linearOverride, theta);
    }
}
