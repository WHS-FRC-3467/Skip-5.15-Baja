// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.FieldConstants.ReefSide;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import lombok.Setter;

public class RobotState {
    private static RobotState instance;

    @Setter
    private double elevatorHeight = 0.0;

    @Setter
    private Pose2d robotPose = Pose2d.kZero;

    @Setter
    private ChassisSpeeds robotVelocity = new ChassisSpeeds();

    public static RobotState getInstance()
    {
        if (instance == null)
            instance = new RobotState();
        return instance;
    }

    public double getElevatorExtensionPercent()
    {
        return MathUtil.clamp(
            elevatorHeight
                / ElevatorConstants.kSubSysConstants.kMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold,
            0.0,
            1.0);
    }

    public Pose2d getFuturePose(double predictionSeconds)
    {
        return predictionSeconds == 0.0 ? robotPose
            : robotPose.exp(robotVelocity.toTwist2d(predictionSeconds));
    }

    public Pose2d getNearestReefFace(double predictionSeconds)
    {
        return getFuturePose(predictionSeconds).nearest(List.of(FieldConstants.Reef.centerFaces));
    }

    public Pose2d getNearestReefFace()
    {
        return getNearestReefFace(0.0);
    }

    public Pose2d getNearestReefBranch(ReefSide side, double predictionSeconds)
    {
        return FieldConstants.Reef.branchPositions
            .get(List.of(FieldConstants.Reef.centerFaces)
                .indexOf(getNearestReefFace(predictionSeconds))
                * 2 + (side == ReefSide.LEFT ? 1 : 0))
            .get(FieldConstants.ReefHeight.L1).toPose2d();
    }

    public Pose2d getNearestReefBranch(ReefSide side)
    {
        return getNearestReefBranch(side, 0.0);
    }

    public boolean isAlgaeHigh(double predictionSeconds)
    {
        return List.of(FieldConstants.Reef.centerFaces)
            .indexOf(getNearestReefFace(predictionSeconds))
            % 2 == 0;
    }

    public boolean isAlgaeHigh()
    {
        return isAlgaeHigh(0.0);
    }

    public Pose2d getNearestCoralStation(double predictionSeconds)
    {
        Translation2d futureTranslation = getFuturePose(predictionSeconds).getTranslation();
        if (futureTranslation.getX() > FieldConstants.fieldLength / 2) {
            if (futureTranslation.getY() > FieldConstants.fieldWidth / 2) {
                return FieldConstants.CoralStation.rightCenterFace
                    .rotateAround(FieldConstants.fieldCenter, Rotation2d.k180deg);
            } else {
                return FieldConstants.CoralStation.leftCenterFace
                    .rotateAround(FieldConstants.fieldCenter, Rotation2d.k180deg);
            }
        } else {
            if (futureTranslation.getY() > FieldConstants.fieldWidth / 2) {
                return FieldConstants.CoralStation.leftCenterFace;
            } else {
                return FieldConstants.CoralStation.rightCenterFace;
            }
        }
    }

    public Pose2d getNearestCoralStation()
    {
        return getNearestCoralStation(0.0);
    }

    public Pose2d getNearestLolipop(double predictionSeconds)
    {
        return getFuturePose(predictionSeconds).nearest(FieldConstants.StagingPositions.iceCreams);
    }

    public Pose2d getNearestLolipop()
    {
        return getNearestLolipop(0.0);
    }

    public Pose2d getNearestProcessor(double predictionSeconds)
    {
        return getFuturePose(predictionSeconds).nearest(FieldConstants.Processor.processors);
    }

    public Pose2d getNearestProcessor()
    {
        return getNearestProcessor(0.0);
    }
}
