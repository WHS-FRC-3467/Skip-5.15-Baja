package frc.robot.commands;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.ReefHeight;
import frc.robot.FieldConstants.ReefSide;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Claw.ClawRoller.ClawRoller;
import frc.robot.subsystems.Claw.ClawRollerLaserCAN.ClawRollerLaserCAN;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.FrontLeftLaserCAN.FrontLeftLaserCAN;
import frc.robot.subsystems.FrontRightLaserCAN.FrontRightLaserCAN;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.TunablePIDController;
import frc.robot.util.Util;

public class ScoreCommandFactory {
    private Drive drive;
    private Supplier<Pose2d> robot;

    private Superstructure superstruct;
    private ClawRoller roller;

    private ClawRollerLaserCAN clawLaserCAN;

    private Supplier<ReefHeight> height;
    private ReefSide side;

    private PIDController offsetController =
        new TunablePIDController("ScoreCoral/OffsetController", 0.01, 0, 0);

    private Pose2d approachTarget;
    private Rotation2d targetRotation;

    private BooleanSupplier laserCANTriggered;
    private Rotation2d offsetDirection;

    private LoggedTunableNumber linearAlignOffset =
        new LoggedTunableNumber("ScoreCoral/LinearAlignOffsetMeters", 0.5);
    private LoggedTunableNumber linearAlignTolerance =
        new LoggedTunableNumber("ScoreCoral/LinearAlignToleranceInches", 5);
    private LoggedTunableNumber thetaAlignTolerance =
        new LoggedTunableNumber("ScoreCoral/ThetaAlignToleranceDegrees", 20);
    private LoggedTunableNumber linearApproachDistanceInches =
        new LoggedTunableNumber("ScoreCoral/LinearApproachDistanceInches", 0);
    private LoggedTunableNumber linearApproachTolerance =
        new LoggedTunableNumber("ScoreCoral/LinearApproachToleranceInches", 2);
    private LoggedTunableNumber thetaApproachTolerance =
        new LoggedTunableNumber("ScoreCoral/ThetaApproachToleranceDegrees", 1);
    private LoggedTunableNumber linearRaiseElevatorTolerance =
        new LoggedTunableNumber("ScoreCoral/LinearRaiseElevatorToleranceMeters",
            1);
    private LoggedTunableNumber thetaRaiseElevatorTolerance =
        new LoggedTunableNumber("ScoreCoral/ThetaRaiseElevatorToleranceDegrees", 8);
    private LoggedTunableNumber linearStartShiftingTolerance =
        new LoggedTunableNumber("ScoreCoral/LinearStartShiftingToleranceInches", 12);
    private LoggedTunableNumber scoreDebounce =
        new LoggedTunableNumber("ScoreCoral/ScoreDebounceSeconds", 0.25);

    public static Command scoreCommand(Drive drive,
        Superstructure superstruct, ClawRoller roller, ClawRollerLaserCAN clawLaserCAN,
        FrontLeftLaserCAN frontLeftLaserCAN, FrontRightLaserCAN frontRightLaserCAN,
        Supplier<ReefHeight> height, ReefSide side)
    {
        return scoreCommand(
            drive,
            drive::getPose,
            superstruct,
            roller,
            clawLaserCAN,
            frontLeftLaserCAN,
            frontRightLaserCAN,
            height,
            side);
    }

    public static Command scoreCommand(Drive drive, Supplier<Pose2d> robot,
        Superstructure superstruct, ClawRoller roller, ClawRollerLaserCAN clawLaserCAN,
        FrontLeftLaserCAN frontLeftLaserCAN, FrontRightLaserCAN frontRightLaserCAN,
        Supplier<ReefHeight> height, ReefSide side)
    {
        return new ScoreCommandFactory(
            drive,
            robot,
            superstruct,
            roller,
            clawLaserCAN,
            frontLeftLaserCAN,
            frontRightLaserCAN,
            height,
            side)
                .get();
    }

    private ScoreCommandFactory(Drive drive, Supplier<Pose2d> robot,
        Superstructure superstruct, ClawRoller roller, ClawRollerLaserCAN clawLaserCAN,
        FrontLeftLaserCAN frontLeftLaserCAN, FrontRightLaserCAN frontRightLaserCAN,
        Supplier<ReefHeight> height, ReefSide side)
    {
        this.drive = drive;
        this.robot = robot;
        this.superstruct = superstruct;
        this.roller = roller;
        this.clawLaserCAN = clawLaserCAN;
        this.height = height;
        this.side = side;
        this.laserCANTriggered =
            side == ReefSide.LEFT ? frontLeftLaserCAN.triggered : frontRightLaserCAN.triggered;
    }

    private Command superstructLevel()
    {
        return Commands.select(Map.of(
            ReefHeight.L1,
            superstruct.getDefaultTransitionCommand(Arm.State.LEVEL_1, Elevator.State.LEVEL_1),
            ReefHeight.L2,
            superstruct.getDefaultTransitionCommand(Arm.State.LEVEL_2, Elevator.State.LEVEL_2),
            ReefHeight.L3,
            superstruct.getDefaultTransitionCommand(Arm.State.LEVEL_3, Elevator.State.LEVEL_3),
            ReefHeight.L4,
            superstruct.getDefaultTransitionCommand(Arm.State.LEVEL_4, Elevator.State.LEVEL_4)),
            height);
    }

    private Command get()
    {
        DriveToPose align = new DriveToPose(
            drive,
            () -> Util
                .moveForward(
                    FieldConstants.getNearestReefBranch(
                        robot.get(),
                        side),
                    (Constants.bumperWidth / 2) + linearAlignOffset.get())
                .transformBy(new Transform2d(0.0, 0.0, Rotation2d.k180deg)))
                    .withTolerance(Units.inchesToMeters(linearAlignTolerance.get()),
                        Rotation2d.fromDegrees(thetaAlignTolerance.get()));

        Command updateTarget = Commands.run(() -> {
            double scalarOffset =
                offsetController.calculate(laserCANTriggered.getAsBoolean() ? 0.0 : 1.0, 0.0);
            Translation2d offsetTranslation = new Translation2d(scalarOffset, offsetDirection);

            approachTarget =
                new Pose2d(approachTarget.getTranslation().plus(offsetTranslation), targetRotation);
        }).beforeStarting(() -> offsetController.reset()).until(offsetController::atSetpoint);

        DriveToPose driveApproach = new DriveToPose(
            drive,
            () -> approachTarget)
                .withTolerance(Units.inchesToMeters(linearApproachTolerance.get()),
                    Rotation2d.fromDegrees(thetaApproachTolerance.get()))
                .finishWithinTolerance(false);
        Command approach = Commands.parallel(
            driveApproach.until(() -> updateTarget.isFinished() && driveApproach.withinTolerance()),
            Commands.sequence(
                Commands.waitUntil(
                    () -> driveApproach.withinTolerance(
                        Units.inchesToMeters(linearStartShiftingTolerance.get()),
                        Rotation2d.fromDegrees(thetaApproachTolerance.get()))),
                updateTarget));

        return Commands.sequence(
            Commands.parallel(
                Commands.sequence(align, approach),
                Commands.sequence(Commands
                    .waitUntil(
                        () -> driveApproach.withinTolerance(linearRaiseElevatorTolerance.get(),
                            Rotation2d.fromDegrees(thetaRaiseElevatorTolerance.get()))
                            || align.withinTolerance(linearRaiseElevatorTolerance.get(),
                                Rotation2d.fromDegrees(thetaRaiseElevatorTolerance.get()))),
                    superstructLevel())),
            roller.setStateCommand(ClawRoller.State.SCORE),
            Commands.waitUntil(clawLaserCAN.triggered.debounce(scoreDebounce.get()).negate()))
            .finallyDo(
                (interrupted) -> superstruct
                    .getTransitionCommand(Arm.State.STOW, Elevator.State.STOW)
                    .onlyIf(() -> !interrupted).schedule())
            .beforeStarting(() -> {
                approachTarget = Util
                    .moveForward(FieldConstants.getNearestReefBranch(robot.get(),
                        side),
                        (Constants.bumperWidth / 2) + linearApproachDistanceInches.get())
                    .transformBy(new Transform2d(0.0, 0.0, Rotation2d.k180deg));
                targetRotation = approachTarget.getRotation();
                offsetDirection =
                    targetRotation
                        .rotateBy(
                            (side == ReefSide.LEFT ? Rotation2d.kCCW_90deg : Rotation2d.kCW_90deg));
            });
    }
}
