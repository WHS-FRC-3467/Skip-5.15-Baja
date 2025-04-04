// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.Vision.VisionConstants.*;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.util.Util;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.WindupXboxController;
import frc.robot.util.PPCalcEndpoint.PPCalcEndpoint;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // Driver Controller
    private final WindupXboxController m_driver = new WindupXboxController(0);

    // Autonomous Selector
    private final LoggedDashboardChooser<Command> m_autoChooser;
    private final LoggedDashboardChooser<Boolean> m_flipChooser;

    // Utilities
    public final PPCalcEndpoint ppAuto;

    // Subsystems
    public final Drive m_drive;
    private final Arm m_profiledArm;
    private final Elevator m_profiledElevator;
    private final Climber m_profiledClimber;
    private final ClawRoller m_clawRoller;
    private final Tongue m_tongue;
    private final ClawRollerLaserCAN m_clawRollerLaserCAN;
    private final Superstructure m_superStruct;

    public final Vision m_vision;
    public final LEDSubsystem m_LED;

    public LoggedTunableNumber speedMultiplier =
        new LoggedTunableNumber("Drivebase Speed Multiplier", 1.0);
    private LoggedTunableNumber alignPredictionSeconds =
        new LoggedTunableNumber("Align Prediction Seconds", 0.3);

    // Trigger for algae/coral mode switching
    private Trigger isCoralMode;
    public Trigger hasVision;
    public Trigger hasLaserCAN;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        m_flipChooser =
            new LoggedDashboardChooser<>("Side");

        m_flipChooser.addOption("Right", false);
        m_flipChooser.addDefaultOption("Left", true);

        ppAuto = new PPCalcEndpoint();

        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                m_drive =
                    new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight),
                        this::shouldMirrorPath);

                m_profiledArm = new Arm(new ArmIOTalonFX(), false);
                m_profiledElevator = new Elevator(new ElevatorIOTalonFX(), false);
                m_profiledClimber = new Climber(new ClimberIOTalonFX() {}, false);
                m_clawRoller = new ClawRoller(new ClawRollerIOTalonFX(), false);
                m_tongue = new Tongue(new TongueIOTalonFX(), false);
                m_clawRollerLaserCAN = new ClawRollerLaserCAN(new ClawRollerLaserCANIOReal());
                isCoralMode = new Trigger(m_clawRollerLaserCAN.triggered.debounce(0.25));
                m_vision =
                    new Vision(
                        m_drive,
                        new VisionIOPhotonVision(camera0Name, robotToCamera0),
                        new VisionIOPhotonVision(camera1Name, robotToCamera1));

                // Instantiate LED Subsystem on BAJA only
                if (Constants.getRobot() == RobotType.BAJA) {
                    m_LED = new LEDSubsystem(new LEDSubsystemIOCANdle(),
                        m_clawRoller, m_profiledArm, m_profiledElevator, m_profiledClimber,
                        m_vision, m_clawRollerLaserCAN.triggered, isCoralMode);
                } else {
                    m_LED = null;
                }
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                m_drive =
                    new Drive(
                        new GyroIO() {},
                        new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight),
                        this::shouldMirrorPath);


                m_profiledArm = new Arm(new ArmIOSim(), true);
                m_profiledElevator = new Elevator(new ElevatorIOSim(), true);
                m_profiledClimber = new Climber(new ClimberIOSim(), true);
                m_clawRoller = new ClawRoller(new ClawRollerIOSim(), true);
                m_tongue = new Tongue(new TongueIOSim(), true);
                m_clawRollerLaserCAN = new ClawRollerLaserCAN(new ClawRollerLaserCANIOSim());
                isCoralMode = new Trigger(m_clawRollerLaserCAN.triggered.debounce(0.25));

                // m_vision =
                // new Vision(
                // m_drive,
                // new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, m_drive::getPose),
                // new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, m_drive::getPose));
                m_vision = new Vision(m_drive, new VisionIO() {}, new VisionIO() {});
                m_LED = new LEDSubsystem(new LEDSubsystemIOWPILib(),
                    m_clawRoller, m_profiledArm, m_profiledElevator, m_profiledClimber,
                    m_vision, m_clawRollerLaserCAN.triggered, isCoralMode);

                break;

            default:
                // Replayed robot, disable IO implementations
                m_drive =
                    new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        this::shouldMirrorPath);

                m_profiledArm = new Arm(new ArmIO() {}, true);
                m_profiledElevator = new Elevator(new ElevatorIOSim(), true);
                m_profiledClimber = new Climber(new ClimberIO() {}, true);
                m_clawRoller = new ClawRoller(new ClawRollerIO() {}, true);
                m_tongue = new Tongue(new TongueIO() {}, true);
                m_clawRollerLaserCAN = new ClawRollerLaserCAN(new ClawRollerLaserCANIO() {});
                isCoralMode = new Trigger(m_clawRollerLaserCAN.triggered.debounce(0.25));
                m_vision = new Vision(m_drive, new VisionIO() {}, new VisionIO() {});
                m_LED = new LEDSubsystem(new LEDSubsystemIO() {},
                    m_clawRoller, m_profiledArm, m_profiledElevator, m_profiledClimber,
                    m_vision, m_clawRollerLaserCAN.triggered, isCoralMode);

                break;
        }

        // Fallback Triggers
        hasVision = new Trigger(() -> m_vision.anyCameraConnected);
        hasLaserCAN = new Trigger(m_clawRollerLaserCAN.validMeasurement);
        // Superstructure coordinates Arm and Elevator motions
        m_superStruct = new Superstructure(m_profiledArm, m_profiledElevator);

        // Pathplanner commands
        registerNamedCommands();

        registerSelfTestCommands();

        // Add all PathPlanner autos to dashboard
        m_autoChooser =
            new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        for (String autoName : AutoBuilder.getAllAutoNames()) {
            m_autoChooser.addOption(autoName + " - Mirrored", new PathPlannerAuto(autoName, true));
        }

        // Drivebase characterizations
        m_autoChooser.addOption(
            "Drive Wheel Radius Characterization",
            DriveCommands.wheelRadiusCharacterization(m_drive));

        // Configure the controller button and joystick bindings
        configureControllerBindings();

        // Detect if controllers are missing / Stop multiple warnings
        DriverStation.silenceJoystickConnectionWarning(false);
    }

    private Command joystickDrive()
    {
        return DriveCommands.joystickDrive(
            m_drive,
            () -> -m_driver.getLeftY() * speedMultiplier.getAsDouble(),
            () -> -m_driver.getLeftX() * speedMultiplier.getAsDouble(),
            () -> -m_driver.getRightX() * speedMultiplier.getAsDouble());
    }

    private Command joystickApproach(Supplier<Pose2d> approachPose)
    {
        return new JoystickApproachCommand(
            m_drive,
            () -> -m_driver.getLeftY() * speedMultiplier.getAsDouble(),
            approachPose);
    }

    private Command DescoreAlgae()
    {
        var approachCommand = new JoystickApproachCommand(
            m_drive,
            () -> -m_driver.getLeftY() * speedMultiplier.getAsDouble(),
            () -> FieldConstants.getNearestReefFace(getFuturePose(alignPredictionSeconds.get())));

        return Commands.deadline(
            Commands.sequence(
                m_clawRoller.setStateCommand(ClawRoller.State.ALGAE_FORWARD),
                Commands.either(
                    m_superStruct.getDefaultTransitionCommand(Arm.State.ALGAE_HIGH,
                        Elevator.State.ALGAE_HIGH),
                    m_superStruct.getDefaultTransitionCommand(Arm.State.ALGAE_LOW,
                        Elevator.State.ALGAE_LOW),
                    () -> FieldConstants.isAlgaeHigh(getFuturePose(alignPredictionSeconds.get()))),
                Commands.waitUntil(m_clawRoller.stalled),
                m_superStruct.getDefaultTransitionCommand(Arm.State.STOW,
                    Elevator.State.ALGAE_STOW)),
            (hasVision.getAsBoolean()) ? approachCommand : Commands.none());
    }

    private Command BargeAlgae()
    {
        var strafeCommand = new JoystickStrafeCommand(
            m_drive,
            () -> -m_driver.getLeftX() * speedMultiplier.getAsDouble(),
            () -> m_drive.getPose().nearest(FieldConstants.Barge.bargeLine));

        return Commands.deadline(
            Commands.sequence(
                Commands.waitUntil(() -> strafeCommand.withinTolerance(Units.inchesToMeters(2.0))),
                m_profiledArm.setStateCommand(Arm.State.STOW),
                Commands.waitUntil(() -> m_profiledArm.atPosition(Units.degreesToRotations(5))),
                m_profiledElevator.setStateCommand(Elevator.State.BARGE),
                Commands.waitUntil(m_profiledElevator.launchHeightTrigger),
                m_clawRoller.setStateCommand(ClawRoller.State.ALGAE_REVERSE),
                Commands.waitUntil(m_clawRoller.stopped.negate()),
                Commands.waitSeconds(0.2),
                m_clawRoller.setStateCommand(ClawRoller.State.OFF),
                m_superStruct.getTransitionCommand(Arm.State.STOW,
                    Elevator.State.STOW, Units.degreesToRotations(10), 0.1)),
            strafeCommand);
    }

    private Command driveTest(double speed)
    {
        return DriveCommands.driveTest(m_drive, speed);
    }

    private Command steerTest(double speed)
    {
        return DriveCommands.steerTest(m_drive, speed);
    }

    private Pose2d getFuturePose(double seconds)
    {
        return m_drive.getPose().exp(m_drive.getChassisSpeeds().toTwist2d(seconds));
    }

    /** Button and Command mappings */
    private void configureControllerBindings()
    {
        // Default command, normal alliance-relative drive
        m_drive.setDefaultCommand(joystickDrive());

        // Driver Right Bumper and Coral Mode: Approach Nearest Right-Side Reef Branch
        m_driver
            .rightBumper().and(isCoralMode).and(hasVision)
            .whileTrue(
                joystickApproach(
                    () -> FieldConstants.getNearestReefBranch(
                        getFuturePose(alignPredictionSeconds.get()), ReefSide.RIGHT)));

        // Driver Left Bumper and Coral Mode: Approach Nearest Left-Side Reef Branch
        m_driver
            .leftBumper().and(isCoralMode).and(hasVision)
            .whileTrue(
                joystickApproach(
                    () -> FieldConstants.getNearestReefBranch(
                        getFuturePose(alignPredictionSeconds.get()), ReefSide.LEFT)));

        // Driver Left and Right Bumpers and Algae mode: Descore to horns on nearest reef face
        m_driver
            .leftBumper().and(m_driver.rightBumper()).and(isCoralMode.negate())
            .whileTrue(
                DescoreAlgae());

        // Driver A Button: Send Arm and Elevator to LEVEL_1
        m_driver
            .a().and(isCoralMode)
            .onTrue(
                m_superStruct.getDefaultTransitionCommand(Arm.State.LEVEL_1,
                    Elevator.State.LEVEL_1));

        m_driver
            .a().and(isCoralMode)
            .whileTrue(
                joystickApproach(
                    () -> FieldConstants
                        .getNearestReefFace(getFuturePose(alignPredictionSeconds.get()))
                        .plus(new Transform2d(0, 0, Rotation2d.k180deg))));

        // Driver A Button and Algae mode: Send Arm and Elevator to Ground Intake
        m_driver
            .a().and(isCoralMode.negate()).and(m_driver.rightTrigger().negate())
            .onTrue(
                Commands.sequence(
                    m_superStruct.getDefaultTransitionCommand(Arm.State.ALGAE_GROUND,
                        Elevator.State.STOW),
                    m_clawRoller.setStateCommand(ClawRoller.State.ALGAE_REVERSE),
                    Commands.waitUntil(m_clawRoller.stalled),
                    m_superStruct.getDefaultTransitionCommand(Arm.State.STOW,
                        Elevator.State.STOW)));

        // Driver X Button: Send Arm and Elevator to LEVEL_2
        m_driver
            .x().and(isCoralMode)
            .onTrue(
                m_superStruct.getTransitionCommand(Arm.State.LEVEL_2, Elevator.State.LEVEL_2));

        // Driver X Button and Algae mode: Lollipop Collect
        m_driver
            .x().and(isCoralMode.negate())
            .onTrue(
                Commands.sequence(
                    m_superStruct.getDefaultTransitionCommand(Arm.State.PROCESSOR_SCORE,
                        Elevator.State.ALGAE_LOLLIPOP),
                    m_clawRoller.setStateCommand(ClawRoller.State.ALGAE_FORWARD),
                    Commands.waitUntil(m_clawRoller.stalled),
                    m_superStruct.getDefaultTransitionCommand(Arm.State.STOW,
                        Elevator.State.STOW)));

        // Point towards driverstation for lollipop pickup and drive slower
        m_driver
            .x().and(isCoralMode.negate())
            .whileTrue(
                DriveCommands.joystickDriveAtAngle(
                    m_drive,
                    () -> -m_driver.getLeftY() * speedMultiplier.getAsDouble() * 0.75,
                    () -> -m_driver.getLeftX() * speedMultiplier.getAsDouble() * 0.75,
                    () -> rotateForAlliance(Rotation2d.k180deg)));

        // Driver B Button: Send Arm and Elevator to LEVEL_3
        m_driver
            .b().and(isCoralMode)
            .onTrue(
                m_superStruct.getDefaultTransitionCommand(Arm.State.LEVEL_3,
                    Elevator.State.LEVEL_3));

        // Driver B Button and Algae mode: Send Arm PROCESSOR SCORE
        m_driver
            .b().and(isCoralMode.negate())
            .onTrue(
                Commands.sequence(
                    m_superStruct.getDefaultTransitionCommand(Arm.State.PROCESSOR_SCORE,
                        Elevator.State.STOW)));

        // Point towards processor and drive slower
        m_driver
            .b().and(isCoralMode.negate())
            .whileTrue(
                DriveCommands.joystickDriveAtAngle(
                    m_drive,
                    () -> -m_driver.getLeftY() * speedMultiplier.getAsDouble() * 0.75,
                    () -> -m_driver.getLeftX() * speedMultiplier.getAsDouble() * 0.75,
                    () -> rotateForAlliance(Rotation2d.kCW_90deg)));


        // Driver Y Button: Send Arm and Elevator to LEVEL_4
        m_driver
            .y().and(isCoralMode)
            .onTrue(
                m_superStruct.getDefaultTransitionCommand(Arm.State.LEVEL_4,
                    Elevator.State.LEVEL_4));

        // Driver Y Button: Auto Barge
        m_driver
            .y().and(isCoralMode.negate())
            .onTrue(
                BargeAlgae());

        // Driver Right Trigger: Place Coral or Algae (Should be done once the robot is in position)
        m_driver
            .rightTrigger().and(m_driver.a().negate()).onTrue(
                Commands.either(
                    Commands.sequence(
                        m_clawRoller.setStateCommand(ClawRoller.State.SCORE),
                        Commands.either(
                            Commands.sequence(
                                Commands.waitUntil(m_clawRollerLaserCAN.triggered.negate()),
                                Commands.waitSeconds(0.2),
                                m_clawRoller.setStateCommand(ClawRoller.State.OFF),
                                m_superStruct.getDefaultTransitionCommand(Arm.State.STOW,
                                    Elevator.State.STOW)),

                            Commands.sequence(
                                Commands.waitUntil(m_clawRoller.stalled.negate()
                                    .and(m_clawRoller.stopped.negate())),
                                Commands.waitSeconds(0.2),
                                m_clawRoller.setStateCommand(ClawRoller.State.OFF),
                                m_superStruct.getDefaultTransitionCommand(Arm.State.STOW,
                                    Elevator.State.STOW)),

                            hasLaserCAN)),

                    Commands.either(
                        m_clawRoller.setStateCommand(ClawRoller.State.ALGAE_FORWARD),
                        m_clawRoller.setStateCommand(ClawRoller.State.ALGAE_REVERSE),
                        () -> m_clawRoller.getState() == ClawRoller.State.ALGAE_REVERSE),

                    isCoralMode));

        m_driver
            .rightTrigger().and(m_driver.a())
            .onTrue(
                Commands.sequence(
                    m_clawRoller.setStateCommand(ClawRoller.State.L1_SCORE),
                    Commands.waitUntil(m_clawRollerLaserCAN.triggered.negate()),
                    Commands.waitSeconds(0.2),
                    m_clawRoller.setStateCommand(ClawRoller.State.OFF),
                    m_superStruct.getDefaultTransitionCommand(Arm.State.STOW,
                        Elevator.State.STOW)));

        m_driver
            .leftTrigger()
            .whileTrue(
                Commands.sequence(
                    m_tongue.setStateCommand(Tongue.State.RAISED),
                    m_clawRoller.setStateCommand(ClawRoller.State.INTAKE),
                    m_superStruct.getDefaultTransitionCommand(Arm.State.CORAL_INTAKE,
                        Elevator.State.CORAL_INTAKE),
                    Commands.either(
                        Commands.waitUntil(
                            m_clawRollerLaserCAN.triggered
                                .and(m_tongue.coralContactTrigger) // TODO: CHECK IF NEEDED
                                .and(m_clawRoller.stopped)),
                        Commands.waitUntil(
                            m_tongue.coralContactTrigger
                                .and(m_clawRoller.stopped)),
                        hasLaserCAN), // If lasercan is not valid, don't check it while intaking
                    m_clawRoller.shuffleCommand(),
                    m_clawRoller.setStateCommand(ClawRoller.State.HOLDCORAL)))
            .onFalse(
                Commands.sequence(
                    m_clawRoller.setStateCommand(ClawRoller.State.OFF),
                    m_superStruct.getDefaultTransitionCommand(Arm.State.STOW, Elevator.State.STOW),
                    m_tongue.lowerTongueCommand(),
                    m_driver.rumbleForTime(0.25, 1)));

        // Start climb request and start index sequence
        m_driver
            .back()
            .onTrue(
                Commands.sequence(
                    m_profiledClimber.setClimbRequestCommand(true),
                    m_profiledClimber.indexClimbState()));

        // Deploy climber and move arm out for clearance
        m_profiledClimber.getClimbRequest().and(m_profiledClimber.getClimbStep1())
            .onTrue(
                Commands.sequence(
                    m_profiledClimber.setStateCommand(Climber.State.PREP),
                    m_superStruct.getDefaultTransitionCommand(Arm.State.CLIMB,
                        Elevator.State.STOW)));

        // Retract climber
        m_profiledClimber.getClimbRequest().and(m_profiledClimber.getClimbStep2())
            .onTrue(
                m_profiledClimber.setStateCommand(Climber.State.CLIMB));

        // Manually climb more on hold
        m_driver
            .back().and(m_profiledClimber.getClimbRequest()).and(m_profiledClimber.getClimbStep3())
            .whileTrue(
                m_profiledClimber.setStateCommand(Climber.State.MANUAL_CLIMB))
            .onFalse(m_profiledClimber.setStateCommand(Climber.State.HOLD));

        // Slow drivetrain to 75% while climbing
        m_profiledClimber.getClimbRequest().whileTrue(
            DriveCommands.joystickDrive(
                m_drive,
                () -> -m_driver.getLeftY() * speedMultiplier.getAsDouble() * .75,
                () -> -m_driver.getLeftX() * speedMultiplier.getAsDouble() * .75,
                () -> -m_driver.getRightX() * speedMultiplier.getAsDouble() * .75));

        m_driver.povLeft().onTrue(
            Commands.sequence(
                m_profiledElevator.setStateCommand(Elevator.State.STOW),
                m_tongue.setStateCommand(Tongue.State.DOWN),
                m_clawRoller.setStateCommand(State.SCORE)))
            .onFalse(m_clawRoller.setStateCommand(State.OFF)
                .andThen(m_tongue.setStateCommand(Tongue.State.STOW)));

        // Driver POV Right: Reset Climbing Sequence if needed
        m_driver.povRight()
            .onTrue(
                Commands.sequence(
                    m_profiledClimber.resetClimb(),
                    m_superStruct.getTransitionCommand(Arm.State.STOW, Elevator.State.STOW)));

        // Driver POV Down: Zero the Elevator (HOMING)
        m_driver
            .povDown().onTrue(
                Commands.sequence(
                    m_profiledArm.setStateCommand(Arm.State.STOW),
                    m_profiledElevator.getHomeCommand()));

        m_driver
            .povUp().onTrue(
                Commands.parallel(
                    m_profiledElevator.setStateCommand(Elevator.State.LEVEL_2),
                    m_profiledArm.setStateCommand(Arm.State.LEVEL_2)));

        // SmartDashboard.putData("ReefPositions",
        // Commands.runOnce(() -> ppAuto.calculatePPEndpoints(Units.inchesToMeters(19)))
        // .ignoringDisable(true));

        SmartDashboard.putData("Drive To Start",
            new DriveToPose(m_drive, () -> getFirstAutoPose().orElse(m_drive.getPose()),
                Units.inchesToMeters(1), Units.inchesToMeters(1), 1));

        SmartDashboard.putData("Drive to Reef", new DriveToPose(m_drive,
            () -> Util
                .moveForward(
                    FieldConstants.getNearestReefBranch(getFuturePose(alignPredictionSeconds.get()),
                        ReefSide.LEFT),
                    (Constants.bumperWidth / 2) + Units.inchesToMeters(0))
                .transformBy(new Transform2d(Translation2d.kZero, Rotation2d.k180deg)),
            Units.inchesToMeters(1.5), Units.inchesToMeters(1), 0.5));
    }

    /**
     * Register Named commands for use in PathPlanner
     */
    private void registerNamedCommands()
    {
        switch (Constants.currentMode) {
            default:

                // Go to the L4 Position
                NamedCommands.registerCommand(
                    "L4",
                    Commands.sequence(
                        Commands.waitUntil(m_clawRollerLaserCAN.triggered),
                        m_tongue.setStateCommand(Tongue.State.DOWN),
                        m_superStruct.getTransitionCommand(Arm.State.LEVEL_4,
                            Elevator.State.LEVEL_4,
                            Units.degreesToRotations(6),
                            0.8),
                        m_clawRoller.L4ShuffleCommand()));

                NamedCommands.registerCommand("AutoAlignLeft",
                    new DriveToPose(m_drive,
                        () -> Util
                            .moveForward(
                                FieldConstants.getNearestReefBranch(m_drive.getPose(),
                                    shouldMirrorPath() ? ReefSide.RIGHT : ReefSide.LEFT),
                                (Constants.bumperWidth / 2) + Units.inchesToMeters(0))
                            .transformBy(new Transform2d(Translation2d.kZero, Rotation2d.k180deg)),
                        Units.inchesToMeters(1.5), Units.inchesToMeters(1.5), .04).withTimeout(2));

                NamedCommands.registerCommand("AutoAlignRight",
                    new DriveToPose(m_drive,
                        () -> Util
                            .moveForward(
                                FieldConstants.getNearestReefBranch(m_drive.getPose(),
                                    shouldMirrorPath() ? ReefSide.LEFT : ReefSide.RIGHT),
                                (Constants.bumperWidth / 2) + Units.inchesToMeters(0))
                            .transformBy(new Transform2d(Translation2d.kZero, Rotation2d.k180deg)),
                        Units.inchesToMeters(1.5), Units.inchesToMeters(1.5), .04).withTimeout(2));

                // Intake Coral
                NamedCommands.registerCommand(
                    "IntakeCoral",
                    Commands.either(
                        Commands.sequence(
                            m_tongue.setStateCommand(Tongue.State.RAISED),
                            m_superStruct.getTransitionCommand(Arm.State.CORAL_INTAKE,
                                Elevator.State.CORAL_INTAKE, Units.degreesToRotations(10), .2),
                            Commands.repeatingSequence(
                                m_clawRoller.setStateCommand(ClawRoller.State.INTAKE),
                                Commands.waitUntil(m_clawRoller.stalled.debounce(0.1)),
                                m_clawRoller.shuffleCommand())
                                .until(m_clawRollerLaserCAN.triggered
                                    .and(m_clawRoller.stopped.debounce(0.05))),
                            m_clawRoller.shuffleCommand(),
                            m_tongue.lowerTongueCommand()),
                        Commands.sequence(
                            m_clawRoller.shuffleCommand(),
                            m_clawRoller.setStateCommand(ClawRoller.State.HOLDCORAL),
                            m_tongue.setStateCommand(Tongue.State.DOWN)),
                        m_clawRollerLaserCAN.triggered.negate()));

                // Prepare Necessary Subsystems Before Intaking
                NamedCommands.registerCommand(
                    "IntakePrep",
                    Commands.sequence(
                        m_clawRoller.setStateCommand(ClawRoller.State.INTAKE),
                        m_tongue.setStateCommand(Tongue.State.RAISED),
                        m_superStruct.getTransitionCommand(Arm.State.CORAL_INTAKE,
                            Elevator.State.CORAL_INTAKE, Units.degreesToRotations(10), .2)));

                // Score Coral
                NamedCommands.registerCommand(
                    "Score",
                    Commands.sequence(
                        m_tongue.setStateCommand(Tongue.State.DOWN),
                        m_clawRoller.setStateCommand(ClawRoller.State.SCORE),
                        Commands.waitUntil(m_clawRollerLaserCAN.triggered.negate()),
                        Commands.waitSeconds(0.2),
                        m_clawRoller.setStateCommand(ClawRoller.State.OFF)));

                // Move to Stow
                NamedCommands.registerCommand(
                    "Stow",
                    Commands.sequence(
                        m_superStruct.getTransitionCommand(Arm.State.STOW,
                            Elevator.State.STOW, Units.degreesToRotations(10), .2)));

                NamedCommands.registerCommand(
                    "PrepScore",
                    Commands.sequence(
                        m_tongue.setStateCommand(Tongue.State.DOWN),
                        m_superStruct.getTransitionCommand(Arm.State.STOW,
                            Elevator.State.LEVEL_3, Units.degreesToRotations(10), .2)));
                break;
            case REPLAY:

                NamedCommands.registerCommand(
                    "L4",
                    Commands.sequence(
                        m_tongue.setStateCommand(Tongue.State.DOWN),
                        m_superStruct.getTransitionCommand(Arm.State.LEVEL_4,
                            Elevator.State.LEVEL_4,
                            Units.degreesToRotations(10),
                            0.8),
                        m_clawRoller.L4ShuffleCommand()));

                NamedCommands.registerCommand("AutoAlignLeft",
                    new DriveToPose(m_drive,
                        () -> Util
                            .moveForward(
                                FieldConstants.getNearestReefBranch(m_drive.getPose(),
                                    shouldMirrorPath() ? ReefSide.RIGHT : ReefSide.LEFT),
                                (Constants.bumperWidth / 2) + Units.inchesToMeters(0))
                            .transformBy(new Transform2d(Translation2d.kZero, Rotation2d.k180deg)),
                        Units.inchesToMeters(1.5), Units.inchesToMeters(1.5), .02).withTimeout(2));

                NamedCommands.registerCommand("AutoAlignRight",
                    new DriveToPose(m_drive,
                        () -> Util
                            .moveForward(
                                FieldConstants.getNearestReefBranch(m_drive.getPose(),
                                    shouldMirrorPath() ? ReefSide.LEFT : ReefSide.RIGHT),
                                (Constants.bumperWidth / 2) + Units.inchesToMeters(0))
                            .transformBy(new Transform2d(Translation2d.kZero, Rotation2d.k180deg)),
                        Units.inchesToMeters(1.5), Units.inchesToMeters(1.5), .02).withTimeout(2));

                // Intake Coral
                NamedCommands.registerCommand(
                    "IntakeCoral",
                    Commands.sequence(
                        m_tongue.setStateCommand(Tongue.State.RAISED),
                        m_superStruct.getTransitionCommand(Arm.State.CORAL_INTAKE,
                            Elevator.State.CORAL_INTAKE, Units.degreesToRotations(10), .2),
                        m_clawRoller.shuffleCommand(),
                        m_tongue.lowerTongueCommand()));

                // Prepare Necessary Subsystems Before Intaking
                NamedCommands.registerCommand(
                    "IntakePrep",
                    Commands.sequence(
                        m_clawRoller.setStateCommand(ClawRoller.State.INTAKE),
                        m_tongue.setStateCommand(Tongue.State.RAISED),
                        m_superStruct.getTransitionCommand(Arm.State.CORAL_INTAKE,
                            Elevator.State.CORAL_INTAKE, Units.degreesToRotations(10), .2)));

                // Score Coral
                NamedCommands.registerCommand(
                    "Score",
                    Commands.sequence(
                        m_clawRoller.setStateCommand(ClawRoller.State.SCORE),
                        m_clawRoller.setStateCommand(ClawRoller.State.OFF)));

                NamedCommands.registerCommand(
                    "Stow",
                    Commands.sequence(
                        m_superStruct.getTransitionCommand(Arm.State.STOW,
                            Elevator.State.STOW, Units.degreesToRotations(10), .2)));
                break;
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return m_autoChooser.get();
    }

    public Optional<Pose2d> getFirstAutoPose()
    {
        var autoCommandName = getAutonomousCommand().getName();
        if (AutoBuilder.getAllAutoNames().contains(autoCommandName)) {
            try {
                List<PathPlannerPath> pathGroup =
                    PathPlannerAuto.getPathGroupFromAutoFile(autoCommandName);

                var firstPath = pathGroup.get(0);
                if (DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red) {
                    firstPath = firstPath.flipPath();
                }
                if (m_flipChooser.get()) {
                    firstPath = firstPath.mirrorPath();
                }
                return Optional.of(new Pose2d(firstPath.getPathPoses().get(0).getTranslation(),
                    firstPath.getIdealStartingState().rotation()));
            } catch (Exception e) {
                return Optional.empty();
            }
        }
        return Optional.empty();
    }

    public Command zeroTongue()
    {
        return m_tongue.zeroSensorCommand();
    }

    public Boolean shouldMirrorPath()
    {
        return m_flipChooser.get();
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
        if (!m_clawRollerLaserCAN.triggered.getAsBoolean()) {
            m_tongue.lowerTongueCommand().schedule();
        }
    }

    public void registerSelfTestCommands()
    {
        // Go to the L3 Position
        NamedCommands.registerCommand(
            "L3",
            Commands.sequence(
                Commands.waitUntil(m_clawRollerLaserCAN.triggered),
                m_tongue.setStateCommand(Tongue.State.DOWN),
                m_superStruct.getTransitionCommand(Arm.State.LEVEL_3, Elevator.State.LEVEL_3,
                    Units.degreesToRotations(10),
                    0.8),
                Commands.waitSeconds(0.25)));

        // Go to the L2 Position
        NamedCommands.registerCommand(
            "L2",
            Commands.sequence(
                Commands.waitUntil(m_clawRollerLaserCAN.triggered),
                m_tongue.setStateCommand(Tongue.State.DOWN),
                m_superStruct.getTransitionCommand(Arm.State.LEVEL_2, Elevator.State.LEVEL_2,
                    Units.degreesToRotations(10),
                    0.8),
                Commands.waitSeconds(0.25)));

        NamedCommands.registerCommand(
            "AlgaeGround",
            Commands.sequence(
                m_superStruct.getTransitionCommand(Arm.State.ALGAE_GROUND,
                    Elevator.State.STOW),
                m_clawRoller.setStateCommand(ClawRoller.State.ALGAE_REVERSE),
                Commands.waitUntil(m_clawRoller.stalled),
                m_superStruct.getTransitionCommand(Arm.State.STOW, Elevator.State.STOW)));

        NamedCommands.registerCommand(
            "Processor",
            m_superStruct.getTransitionCommand(Arm.State.STOW, Elevator.State.STOW,
                Units.degreesToRotations(10), 0.8));

        // Driver X Button and Algae mode: Send Arm and Elevator to ALGAE_LOW position
        NamedCommands.registerCommand(
            "AlgaeLow",
            Commands.sequence(
                Commands.waitUntil(m_clawRoller.stalled.negate()),
                m_superStruct.getTransitionCommand(Arm.State.ALGAE_LOW,
                    Elevator.State.ALGAE_LOW),
                m_clawRoller.setStateCommand(ClawRoller.State.ALGAE_FORWARD),
                Commands.waitUntil(m_clawRoller.stalled),
                m_superStruct.getTransitionCommand(Arm.State.STOW, Elevator.State.STOW)));

        // Send Arm and Elevator to ALGAE_HIGH position
        NamedCommands.registerCommand(
            "AlgaeHigh",
            Commands.sequence(
                Commands.waitUntil(m_clawRoller.stalled.negate()),
                m_superStruct.getTransitionCommand(Arm.State.ALGAE_HIGH,
                    Elevator.State.ALGAE_HIGH),
                m_clawRoller.setStateCommand(ClawRoller.State.ALGAE_FORWARD),
                Commands.waitUntil(m_clawRoller.stalled),
                m_superStruct.getTransitionCommand(Arm.State.STOW, Elevator.State.STOW)));

        // Algae Barge SP
        NamedCommands.registerCommand(
            "AlgaeBarge",
            Commands.sequence(
                m_superStruct.getTransitionCommand(Arm.State.BARGE, Elevator.State.BARGE)));

        // Release Algae
        NamedCommands.registerCommand(
            "AlgaeScore",
            Commands.sequence(
                Commands.either(m_clawRoller.setStateCommand(ClawRoller.State.ALGAE_FORWARD),
                    m_clawRoller.setStateCommand(ClawRoller.State.ALGAE_REVERSE),
                    () -> m_clawRoller.getState() == ClawRoller.State.ALGAE_REVERSE),
                Commands.waitUntil(m_clawRoller.stalled.negate()),
                Commands.waitSeconds(1),
                m_clawRoller.setStateCommand(ClawRoller.State.OFF),
                m_superStruct.getTransitionCommand(Arm.State.STOW, Elevator.State.STOW)));

        // Tells drivebase to drive at 1 meter per second, no turning
        NamedCommands.registerCommand(
            "DriveTest",
            driveTest(1.0));

        // Tells drivebase to rotate at 1 radian per second, no turning
        NamedCommands.registerCommand(
            "SteerTest",
            steerTest(1.0));
    }
}
