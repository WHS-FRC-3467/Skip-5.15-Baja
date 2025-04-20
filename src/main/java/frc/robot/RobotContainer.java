// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.Vision.VisionConstants.*;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.RobotType;
import frc.robot.FieldConstants.ReefSide;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.JoystickApproachCommand;
import frc.robot.commands.JoystickStrafeCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Arm.*;
import frc.robot.subsystems.Claw.ClawRoller.ClawRoller;
import frc.robot.subsystems.Claw.ClawRoller.ClawRollerIO;
import frc.robot.subsystems.Claw.ClawRoller.ClawRollerIOSim;
import frc.robot.subsystems.Claw.ClawRoller.ClawRollerIOTalonFX;
import frc.robot.subsystems.Claw.ClawRoller.ClawRoller.State;
import frc.robot.subsystems.Claw.ClawRollerLaserCAN.ClawRollerLaserCAN;
import frc.robot.subsystems.Claw.ClawRollerLaserCAN.ClawRollerLaserCANIO;
import frc.robot.subsystems.Claw.ClawRollerLaserCAN.ClawRollerLaserCANIOReal;
import frc.robot.subsystems.Claw.ClawRollerLaserCAN.ClawRollerLaserCANIOSim;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.ClimberIO;
import frc.robot.subsystems.Climber.ClimberIOSim;
import frc.robot.subsystems.Climber.ClimberIOTalonFX;
import frc.robot.subsystems.Elevator.*;
import frc.robot.subsystems.LED.LEDSubsystem;
import frc.robot.subsystems.LED.LEDSubsystemIO;
import frc.robot.subsystems.LED.LEDSubsystemIOCANdle;
import frc.robot.subsystems.LED.LEDSubsystemIOWPILib;
import frc.robot.subsystems.Tongue.Tongue;
import frc.robot.subsystems.Tongue.TongueIO;
import frc.robot.subsystems.Tongue.TongueIOSim;
import frc.robot.subsystems.Tongue.TongueIOTalonFX;
import frc.robot.subsystems.Vision.*;
import frc.robot.subsystems.drive.*;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.WindupXboxController;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // Driver Controller
    private final WindupXboxController driver = new WindupXboxController(0).withDeadband(0.1);

    // Subsystems
    public final Drive drive;
    private final Arm arm;
    private final Elevator elevator;
    private final Climber climber;
    private final ClawRoller clawRoller;
    private final Tongue tongue;
    private final ClawRollerLaserCAN clawRollerLaserCAN;
    private final Superstructure superStruct;

    public final Vision vision;
    public final LEDSubsystem LED;

    private LoggedTunableNumber alignPredictionSeconds =
        new LoggedTunableNumber("Align Prediction Seconds", 0.3);

    // Trigger for algae/coral mode switching
    @AutoLogOutput
    private Trigger isCoralMode;

    // Override Triggers
    public Trigger hasVision;
    public Trigger hasLaserCAN;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {

        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive =
                    new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));

                arm = new Arm(new ArmIOTalonFX(), false);
                elevator = new Elevator(new ElevatorIOTalonFX(), false);
                if (Constants.getRobot() == RobotType.BAJA) {
                    climber = new Climber(new ClimberIOTalonFX(), false);
                } else {
                    climber = new Climber(new ClimberIO() {}, false);
                }
                clawRoller = new ClawRoller(new ClawRollerIOTalonFX(), false);
                tongue = new Tongue(new TongueIOTalonFX(), false);
                clawRollerLaserCAN = new ClawRollerLaserCAN(new ClawRollerLaserCANIOReal());
                isCoralMode = new Trigger(clawRollerLaserCAN.triggered.debounce(0.25));
                vision =
                    new Vision(
                        drive,
                        new VisionIOPhotonVision(camera0Name, robotToCamera0),
                        new VisionIOPhotonVision(camera1Name, robotToCamera1));

                // Instantiate LED Subsystem on BAJA only
                if (Constants.getRobot() == RobotType.BAJA) {
                    LED = new LEDSubsystem(new LEDSubsystemIOCANdle(),
                        clawRoller, arm, elevator, climber,
                        vision, clawRollerLaserCAN.triggered, isCoralMode);
                } else {
                    LED = null;
                }

                hasVision = new Trigger(() -> vision.anyCameraConnected);
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drive =
                    new Drive(
                        new GyroIO() {},
                        new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight));

                arm = new Arm(new ArmIOSim(), true);
                elevator = new Elevator(new ElevatorIOSim(), true);
                climber = new Climber(new ClimberIOSim(), true);
                clawRoller = new ClawRoller(new ClawRollerIOSim(), true);
                tongue = new Tongue(new TongueIOSim(), true);
                clawRollerLaserCAN = new ClawRollerLaserCAN(new ClawRollerLaserCANIOSim());
                isCoralMode = new Trigger(clawRollerLaserCAN.triggered.debounce(0.25));

                vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
                LED = new LEDSubsystem(new LEDSubsystemIOWPILib(),
                    clawRoller, arm, elevator, climber,
                    vision, clawRollerLaserCAN.triggered, isCoralMode);

                hasVision = new Trigger(() -> true);
                break;

            default:
                // Replayed robot, disable IO implementations
                drive =
                    new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {});

                arm = new Arm(new ArmIO() {}, true);
                elevator = new Elevator(new ElevatorIOSim(), true);
                climber = new Climber(new ClimberIO() {}, true);
                clawRoller = new ClawRoller(new ClawRollerIO() {}, true);
                tongue = new Tongue(new TongueIO() {}, true);
                clawRollerLaserCAN = new ClawRollerLaserCAN(new ClawRollerLaserCANIO() {});
                isCoralMode = new Trigger(clawRollerLaserCAN.triggered.debounce(0.25));
                vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
                LED = new LEDSubsystem(new LEDSubsystemIO() {}, clawRoller, arm,
                    elevator, climber, vision, clawRollerLaserCAN.triggered,
                    isCoralMode);

                hasVision = new Trigger(() -> vision.anyCameraConnected);
                break;
        }

        // Fallback Triggers
        hasLaserCAN = new Trigger(clawRollerLaserCAN.validMeasurement);

        // Superstructure coordinates Arm and Elevator motions
        superStruct = new Superstructure(arm, elevator);

        // Configure the controller button and joystick bindings
        configureControllerBindings();

        // Detect if controllers are missing / Stop multiple warnings
        DriverStation.silenceJoystickConnectionWarning(false);
    }

    private Command joystickDrive()
    {
        return DriveCommands.joystickDrive(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX());
    }

    private JoystickApproachCommand joystickApproach(Supplier<Pose2d> approachPose)
    {
        return new JoystickApproachCommand(
            drive,
            () -> driver.getLeftY(),
            approachPose);
    }

    private Command descoreAlgae()
    {
        var approachCommand = new JoystickApproachCommand(
            drive,
            () -> driver.getLeftY(),
            () -> FieldConstants.getNearestReefFace(getFuturePose(alignPredictionSeconds.get())));

        return Commands.deadline(
            Commands.sequence(
                clawRoller.setStateCommand(ClawRoller.State.ALGAE_FORWARD),
                Commands.either(
                    superStruct.getDefaultTransitionCommand(Arm.State.ALGAE_HIGH,
                        Elevator.State.ALGAE_HIGH),
                    superStruct.getDefaultTransitionCommand(Arm.State.ALGAE_LOW,
                        Elevator.State.ALGAE_LOW),
                    () -> FieldConstants.isAlgaeHigh(getFuturePose(alignPredictionSeconds.get()))),
                Commands.waitUntil(clawRoller.stalled),
                superStruct.getDefaultTransitionCommand(Arm.State.STOW,
                    Elevator.State.ALGAE_STOW)),
            (hasVision.getAsBoolean()) ? approachCommand : Commands.none());
    }

    private Command descoreAlgaeProcessor()
    {
        var approachCommand = new JoystickApproachCommand(
            drive,
            () -> driver.getLeftY(),
            () -> FieldConstants.getNearestReefFace(getFuturePose(alignPredictionSeconds.get())));

        return Commands.deadline(
            Commands.sequence(
                clawRoller.setStateCommand(ClawRoller.State.ALGAE_REVERSE),
                Commands.either(
                    superStruct.getDefaultTransitionCommand(Arm.State.ALGAE_HIGH_P,
                        Elevator.State.ALGAE_HIGH_P),
                    superStruct.getDefaultTransitionCommand(Arm.State.ALGAE_LOW_P,
                        Elevator.State.ALGAE_LOW_P),
                    () -> FieldConstants.isAlgaeHigh(getFuturePose(alignPredictionSeconds.get()))),
                Commands.waitUntil(clawRoller.stalled),
                superStruct.getDefaultTransitionCommand(Arm.State.STOW,
                    Elevator.State.ALGAE_STOW)),
            (hasVision.getAsBoolean()) ? approachCommand : Commands.none());
    }

    private Command BargeAlgae()
    {
        var strafeCommand = new JoystickStrafeCommand(
            drive,
            () -> -driver.getLeftX(),
            () -> drive.getPose().nearest(FieldConstants.Barge.bargeLine));

        return Commands.deadline(
            Commands.sequence(
                Commands.waitUntil(
                    () -> strafeCommand.withinTolerance(
                        Units.inchesToMeters(2.0),
                        Rotation2d.fromDegrees(4.0))),
                arm.setStateCommand(Arm.State.STOW),
                Commands.waitUntil(() -> arm.atPosition(Units.degreesToRotations(10))),
                elevator.setStateCommand(Elevator.State.BARGE),
                Commands.waitUntil(elevator.launchHeightTrigger),
                clawRoller.setStateCommand(ClawRoller.State.ALGAE_REVERSE),
                Commands.waitUntil(clawRoller.stopped.negate()),
                Commands.waitSeconds(0.2),
                clawRoller.setStateCommand(ClawRoller.State.OFF)),
            strafeCommand)
            .finallyDo(interrupted -> {
                if (!interrupted)
                    superStruct.getTransitionCommand(Arm.State.STOW,
                        Elevator.State.STOW, Units.degreesToRotations(10), 0.1).schedule();
            });
    }

    private Pose2d getFuturePose(double seconds)
    {
        return drive.getPose().exp(drive.getChassisSpeeds().toTwist2d(seconds));
    }

    /** Button and Command mappings */
    private void configureControllerBindings()
    {
        // Default command, normal alliance-relative drive
        drive.setDefaultCommand(joystickDrive());

        // Driver Right Bumper and Coral Mode: Approach Nearest Right-Side Reef Branch
        driver
            .rightBumper().and(hasVision)
            .whileTrue(
                Commands.either(
                    joystickApproach(
                        () -> FieldConstants.getNearestReefBranch(
                            getFuturePose(alignPredictionSeconds.get()), ReefSide.RIGHT))
                                .until(isCoralMode.negate()),
                    Commands.none(),
                    isCoralMode));

        // Driver Left Bumper and Coral Mode: Approach Nearest Left-Side Reef Branch
        driver
            .leftBumper().and(hasVision)
            .whileTrue(
                Commands.either(
                    joystickApproach(
                        () -> FieldConstants.getNearestReefBranch(
                            getFuturePose(alignPredictionSeconds.get()), ReefSide.LEFT))
                                .until(isCoralMode.negate()),
                    Commands.none(),
                    isCoralMode));

        // Driver Left and Right Bumpers and Algae mode: Descore to horns on nearest reef face
        driver
            .leftBumper().and(driver.rightBumper()).and(isCoralMode.negate())
            .whileTrue(descoreAlgae());

        driver
            .leftBumper().and(driver.a())
            .whileTrue(spitAndStrafe(ReefSide.LEFT));

        driver
            .rightBumper().and(driver.a())
            .whileTrue(spitAndStrafe(ReefSide.RIGHT));

        driver
            .a()
            .onTrue(
                Commands.either(
                    // Driver A Button: Send Arm and Elevator to LEVEL_1
                    superStruct.getDefaultTransitionCommand(Arm.State.LEVEL_1,
                        Elevator.State.LEVEL_1),
                    // Driver A Button and Algae mode: Send Arm and Elevator to Ground Intake
                    Commands.sequence(
                        superStruct.getDefaultTransitionCommand(Arm.State.ALGAE_GROUND,
                            Elevator.State.CORAL_INTAKE),
                        clawRoller.setStateCommand(ClawRoller.State.ALGAE_REVERSE),
                        Commands.waitUntil(clawRoller.stalled),
                        superStruct.getDefaultTransitionCommand(Arm.State.STOW,
                            Elevator.State.STOW)),
                    isCoralMode))
            .whileTrue(
                Commands.either(
                    DriveCommands.joystickDriveAtAngle(
                        drive,
                        () -> -driver.getLeftY(),
                        () -> -driver.getLeftX(),
                        () -> FieldConstants
                            .getNearestReefFace(getFuturePose(alignPredictionSeconds.get()))
                            .getRotation().plus(Rotation2d.k180deg)),
                    Commands.none(),
                    isCoralMode));

        driver
            .start()
            .whileTrue(descoreAlgaeProcessor());

        // Driver X Button: Send Arm and Elevator to LEVEL_2
        driver
            .x().and(isCoralMode)
            .onTrue(
                superStruct.getDefaultTransitionCommand(Arm.State.LEVEL_2,
                    Elevator.State.LEVEL_2));

        // Driver X Button and Algae mode: Lollipop Collect
        driver
            .x().and(isCoralMode.negate())
            .onTrue(
                Commands.sequence(
                    superStruct.getDefaultTransitionCommand(Arm.State.PROCESSOR_SCORE,
                        Elevator.State.ALGAE_LOLLIPOP),
                    clawRoller.setStateCommand(ClawRoller.State.ALGAE_FORWARD),
                    Commands.waitUntil(clawRoller.stalled),
                    superStruct.getDefaultTransitionCommand(Arm.State.STOW,
                        Elevator.State.STOW)));

        // Point towards driverstation for lollipop pickup and drive slower
        driver
            .x().and(isCoralMode.negate())
            .whileTrue(
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -driver.getLeftY() * 0.75,
                    () -> -driver.getLeftX() * 0.75,
                    () -> rotateForAlliance(Rotation2d.k180deg)));

        // Driver B Button: Send Arm and Elevator to LEVEL_3
        driver
            .b().and(isCoralMode)
            .onTrue(
                superStruct.getDefaultTransitionCommand(Arm.State.LEVEL_3,
                    Elevator.State.LEVEL_3));

        // Driver B Button and Algae mode: Send Arm PROCESSOR SCORE
        driver
            .b().and(isCoralMode.negate())
            .onTrue(
                Commands.sequence(
                    superStruct.getDefaultTransitionCommand(Arm.State.PROCESSOR_SCORE,
                        Elevator.State.STOW)));

        // Point towards processor and drive slower
        driver
            .b().and(isCoralMode.negate())
            .whileTrue(
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -driver.getLeftY() * 0.75,
                    () -> -driver.getLeftX() * 0.75,
                    () -> rotateForAlliance(Rotation2d.kCW_90deg)));


        // Driver Y Button: Send Arm and Elevator to LEVEL_4
        driver
            .y().and(isCoralMode)
            .onTrue(
                superStruct.getDefaultTransitionCommand(Arm.State.LEVEL_4,
                    Elevator.State.LEVEL_4));

        // Driver Y Button: Auto Barge
        driver
            .y().and(isCoralMode.negate())
            .onTrue(
                BargeAlgae());

        // Driver Right Trigger: Place Coral or Algae (Should be done once the robot is in position)
        driver
            .rightTrigger().and(driver.a().negate()).onTrue(
                Commands.either(
                    Commands.sequence(
                        clawRoller.setStateCommand(ClawRoller.State.SCORE),
                        Commands.either(
                            Commands.sequence(
                                Commands.waitUntil(clawRollerLaserCAN.triggered.negate()),
                                Commands.waitSeconds(0.2),
                                clawRoller.setStateCommand(ClawRoller.State.OFF),
                                superStruct.getDefaultTransitionCommand(Arm.State.STOW,
                                    Elevator.State.STOW)),

                            Commands.sequence(
                                Commands.waitUntil(clawRoller.stalled.negate()
                                    .and(clawRoller.stopped.negate())),
                                Commands.waitSeconds(0.2),
                                clawRoller.setStateCommand(ClawRoller.State.OFF),
                                superStruct.getDefaultTransitionCommand(Arm.State.STOW,
                                    Elevator.State.STOW)),

                            hasLaserCAN)),

                    Commands.either(
                        clawRoller.setStateCommand(ClawRoller.State.ALGAE_FORWARD),
                        clawRoller.setStateCommand(ClawRoller.State.ALGAE_REVERSE),
                        () -> clawRoller.getState() == ClawRoller.State.ALGAE_REVERSE),

                    isCoralMode));

        driver
            .leftTrigger()
            .whileTrue(
                Commands.sequence(
                    tongue.setStateCommand(Tongue.State.RAISED),
                    superStruct.getDefaultTransitionCommand(Arm.State.CORAL_INTAKE,
                        Elevator.State.CORAL_INTAKE),
                    Commands.repeatingSequence(
                        clawRoller.setStateCommand(ClawRoller.State.INTAKE),
                        Commands.waitUntil(clawRoller.stalled.debounce(0.1)),
                        clawRoller.shuffleCommand())
                        .until(clawRollerLaserCAN.triggered
                            .and(clawRoller.stopped.debounce(0.15))),
                    Commands.either(
                        Commands.waitUntil(
                            clawRollerLaserCAN.triggered
                                .and(tongue.coralContactTrigger)
                                .and(clawRoller.stopped)),
                        Commands.waitUntil(
                            tongue.coralContactTrigger
                                .and(clawRoller.stopped)),
                        hasLaserCAN), // If lasercan is not valid, don't check it while intaking
                    clawRoller.shuffleCommand(),
                    clawRoller.setStateCommand(ClawRoller.State.HOLDCORAL)))
            .onFalse(
                Commands.sequence(
                    clawRoller.setStateCommand(ClawRoller.State.OFF),
                    superStruct.getDefaultTransitionCommand(Arm.State.STOW, Elevator.State.STOW),
                    tongue.lowerTongueCommand(),
                    driver.rumbleForTime(0.25, 1)));

        // Start climb request and start index sequence
        driver
            .back()
            .onTrue(
                Commands.sequence(
                    climber.setClimbRequestCommand(true),
                    climber.indexClimbState()));

        // Deploy climber and move arm out for clearance
        climber.getClimbRequest().and(climber.getClimbStep1())
            .onTrue(
                Commands.sequence(
                    climber.setStateCommand(Climber.State.PREP),
                    superStruct.getDefaultTransitionCommand(Arm.State.CLIMB,
                        Elevator.State.STOW)));

        // Retract climber
        climber.getClimbRequest().and(climber.getClimbStep2())
            .onTrue(
                climber.setStateCommand(Climber.State.CLIMB));

        // Manually climb more on hold
        driver
            .back().and(climber.getClimbRequest()).and(climber.getClimbStep3())
            .whileTrue(
                climber.setStateCommand(Climber.State.MANUAL_CLIMB))
            .onFalse(climber.setStateCommand(Climber.State.HOLD));

        // Slow drivetrain to 75% while climbing
        climber.getClimbRequest().whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -driver.getLeftY() * .75,
                () -> -driver.getLeftX() * .75,
                () -> -driver.getRightX() * .75));

        driver.povLeft().onTrue(
            Commands.sequence(
                elevator.setStateCommand(Elevator.State.STOW),
                tongue.setStateCommand(Tongue.State.DOWN),
                clawRoller.setStateCommand(State.SCORE)))
            .onFalse(clawRoller.setStateCommand(State.OFF)
                .andThen(tongue.setStateCommand(Tongue.State.STOW)));

        // Driver POV Right: Reset Climbing Sequence if needed
        driver.povRight()
            .onTrue(
                Commands.sequence(
                    climber.resetClimb(),
                    superStruct.getTransitionCommand(Arm.State.STOW, Elevator.State.STOW)));

        // Driver POV Down: Zero the Elevator (HOMING)
        driver
            .povDown().onTrue(
                Commands.sequence(
                    arm.setStateCommand(Arm.State.STOW),
                    elevator.getHomeCommand()));

        driver
            .povUp().onTrue(
                Commands.parallel(
                    elevator.setStateCommand(Elevator.State.LEVEL_3),
                    arm.setStateCommand(Arm.State.LEVEL_2)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return Commands.none();
    }

    public Command zeroTongue()
    {
        return tongue.zeroSensorCommand();
    }

    public Rotation2d rotateForAlliance(Rotation2d target)
    {
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                return target.rotateBy(Rotation2d.k180deg);
            } else {
                return target;
            }
        } else {
            return target;
        }
    }

    public void lowerTongueTele()
    {
        if (!clawRollerLaserCAN.triggered.getAsBoolean()) {
            tongue.lowerTongueCommand().schedule();
        }
    }

    public Command spitAndStrafe(ReefSide side)
    {
        return Commands.deadline(
            Commands.sequence(

                clawRoller.L1ShuffleCommand(),
                tongue.setStateCommand(Tongue.State.L1),
                Commands.waitSeconds(0.125),
                clawRoller.setStateCommand(ClawRoller.State.L1_SCORE),
                Commands.waitUntil(clawRollerLaserCAN.triggered.negate()),
                Commands.waitSeconds(0.25)),

            Commands.either(
                new DriveToPose(
                    drive,
                    () -> FieldConstants.getNearestReefBranch(
                        getFuturePose(alignPredictionSeconds.get()), side)
                        .transformBy(new Transform2d(
                            Constants.bumperWidth / 2 - Units.inchesToMeters(1),
                            Units.inchesToMeters(24),
                            Rotation2d.k180deg))),
                new DriveToPose(
                    drive,
                    () -> FieldConstants.getNearestReefBranch(
                        getFuturePose(alignPredictionSeconds.get()), side)
                        .transformBy(new Transform2d(
                            Constants.bumperWidth / 2 - Units.inchesToMeters(1),
                            Units.inchesToMeters(-24),
                            Rotation2d.k180deg))),
                () -> side == ReefSide.RIGHT));
    }
}
