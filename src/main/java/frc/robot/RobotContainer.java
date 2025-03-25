// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.Vision.VisionConstants.*;
import java.util.function.Supplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.RobotType;
import frc.robot.FieldConstants.ReefSide;
import frc.robot.commands.DriveCommands;
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
import frc.robot.subsystems.Tounge.Tounge;
import frc.robot.subsystems.Tounge.ToungeIO;
import frc.robot.subsystems.Tounge.ToungeIOSim;
import frc.robot.subsystems.Tounge.ToungeIOTalonFX;
import frc.robot.subsystems.Vision.*;
import frc.robot.subsystems.drive.*;
import frc.robot.util.WindupXboxController;
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

    // Subsystems
    public final Drive m_drive;
    private final Arm m_profiledArm;
    private final Elevator m_profiledElevator;
    private final Climber m_profiledClimber;
    private final ClawRoller m_clawRoller;
    private final Tounge m_tounge;
    private final ClawRollerLaserCAN m_clawRollerLaserCAN;
    private final Superstructure m_superStruct;

    public final Vision m_vision;
    public final LEDSubsystem m_LED;

    // Trigger for algae/coral mode switching
    private boolean coralModeEnabled = true;
    private boolean isProcessorModeEnabled = true;
    private Trigger isCoralMode = new Trigger(() -> coralModeEnabled);
    private Trigger isProcessorMode = new Trigger(() -> isProcessorModeEnabled);


    private double speedMultiplier = 0.9;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                m_drive =
                    new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));

                m_profiledArm = new Arm(new ArmIOTalonFX(), false);
                m_profiledElevator = new Elevator(new ElevatorIOTalonFX(), false);
                m_profiledClimber = new Climber(new ClimberIOTalonFX(), false);
                m_clawRoller = new ClawRoller(new ClawRollerIOTalonFX(), false);
                m_tounge = new Tounge(new ToungeIOTalonFX(), false);
                m_clawRollerLaserCAN = new ClawRollerLaserCAN(new ClawRollerLaserCANIOReal());

                m_vision =
                    new Vision(
                        m_drive,
                        new VisionIOPhotonVision(camera0Name, robotToCamera0),
                        new VisionIOPhotonVision(camera1Name, robotToCamera1));

                // Instantiate LED Subsystem on BAJA only
                if (Constants.getRobot() == RobotType.BAJA) {
                    m_LED = new LEDSubsystem(new LEDSubsystemIOCANdle(),
                        m_clawRoller, m_profiledArm, m_profiledElevator, m_profiledClimber,
                        m_vision, m_clawRollerLaserCAN.triggered, isCoralMode, isProcessorMode);
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
                        new ModuleIOSim(TunerConstants.BackRight));


                m_profiledArm = new Arm(new ArmIOSim(), true);
                m_profiledElevator = new Elevator(new ElevatorIOSim(), true);
                m_profiledClimber = new Climber(new ClimberIOSim(), true);
                m_clawRoller = new ClawRoller(new ClawRollerIOSim(), true);
                m_tounge = new Tounge(new ToungeIOSim(), true);
                m_clawRollerLaserCAN = new ClawRollerLaserCAN(new ClawRollerLaserCANIOSim());

                m_vision =
                    new Vision(
                        m_drive,
                        new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, m_drive::getPose),
                        new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, m_drive::getPose));

                m_LED = new LEDSubsystem(new LEDSubsystemIOWPILib(),
                    m_clawRoller, m_profiledArm, m_profiledElevator, m_profiledClimber,
                    m_vision, m_clawRollerLaserCAN.triggered, isCoralMode, isProcessorMode);

                break;

            default:
                // Replayed robot, disable IO implementations
                m_drive =
                    new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {});

                m_profiledArm = new Arm(new ArmIO() {}, true);
                m_profiledElevator = new Elevator(new ElevatorIOSim(), true);
                m_profiledClimber = new Climber(new ClimberIO() {}, true);
                m_clawRoller = new ClawRoller(new ClawRollerIO() {}, true);
                m_tounge = new Tounge(new ToungeIO() {}, true);
                m_clawRollerLaserCAN = new ClawRollerLaserCAN(new ClawRollerLaserCANIO() {});

                m_vision = new Vision(m_drive, new VisionIO() {}, new VisionIO() {});
                m_LED = new LEDSubsystem(new LEDSubsystemIO() {},
                    m_clawRoller, m_profiledArm, m_profiledElevator, m_profiledClimber,
                    m_vision, m_clawRollerLaserCAN.triggered, isCoralMode, isProcessorMode);
                break;
        }

        // Superstructure coordinates Arm and Elevator motions
        m_superStruct = new Superstructure(m_profiledArm, m_profiledElevator);

        // Pathplanner commands
        registerNamedCommands();

        // Add all PathPlanner autos to dashboard
        m_autoChooser =
            new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Drivebase characterizations
        m_autoChooser.addOption(
            "Drive Wheel Radius Characterization",
            DriveCommands.wheelRadiusCharacterization(m_drive));

        // Configure the controller button and joystick bindings
        configureControllerBindings();

        // Detect if controllers are missing / Stop multiple warnings
        if (Robot.isReal()) {
            DriverStation.silenceJoystickConnectionWarning(false);
        } else {
            DriverStation.silenceJoystickConnectionWarning(true);
        }

    }

    private Command joystickDrive()
    {
        return DriveCommands.joystickDrive(
            m_drive,
            () -> -m_driver.getLeftY() * speedMultiplier,
            () -> -m_driver.getLeftX() * speedMultiplier,
            () -> -m_driver.getRightX());
    }

    private Command joystickApproach(Supplier<Pose2d> approachPose)
    {
        return DriveCommands.joystickApproach(
            m_drive,
            () -> -m_driver.getLeftY() * speedMultiplier,
            approachPose);
    }

    private Command joystickStrafe(Supplier<Pose2d> approachPose)
    {
        return DriveCommands.joystickStrafe(
            m_drive,
            () -> m_driver.getLeftX() * speedMultiplier,
            approachPose);
    }

    public Command setCoralAlgaeModeCommand()
    {
        return Commands.runOnce(
            () -> {
                coralModeEnabled = !coralModeEnabled;
            });
    }

    public Command toggleProcessorMode()
    {
        return Commands.runOnce(
            () -> {
                isProcessorModeEnabled = !isProcessorModeEnabled;
            });
    }

    /** Button and Command mappings */
    private void configureControllerBindings()
    {
        // Default command, normal field-relative drive
        m_drive.setDefaultCommand(joystickDrive());

        // Driver Right Bumper: Approach Nearest Right-Side Reef Branch
        m_driver.rightBumper().and(isCoralMode)
            .whileTrue(
                joystickApproach(
                    () -> FieldConstants.getNearestReefBranch(m_drive.getPose(), ReefSide.RIGHT)));

        // Driver Left Bumper: Approach Nearest Left-Side Reef Branch
        m_driver.leftBumper().and(isCoralMode)
            .whileTrue(
                joystickApproach(
                    () -> FieldConstants.getNearestReefBranch(m_drive.getPose(), ReefSide.LEFT)));

        // Driver Right Bumper and Algae mode: Approach Nearest Reef Face
        m_driver.rightBumper().and(isCoralMode.negate())
            .whileTrue(
                joystickApproach(() -> FieldConstants.getNearestReefFace(m_drive.getPose())));

        // Driver Left Bumper and Algae mode: Approach Nearest Reef Face
        m_driver.leftBumper().and(isCoralMode.negate())
            .whileTrue(
                joystickStrafe(() -> m_drive.getPose().nearest(FieldConstants.Barge.align)));

        // Driver A Button: Send Arm and Elevator to LEVEL_1
        m_driver
            .a().and(isCoralMode)
            .onTrue(
                m_superStruct.getTransitionCommand(Arm.State.LEVEL_1, Elevator.State.LEVEL_1));

        // Driver A Button and Algae mode: Send Arm and Elevator to Ground Intake
        m_driver
            .a().and(isCoralMode.negate())
            .and(m_clawRoller.stalled.negate())
            .onTrue(
                Commands.sequence(
                    m_superStruct.getTransitionCommand(Arm.State.ALGAE_GROUND,
                        Elevator.State.STOW),
                    m_clawRoller.setStateCommand(ClawRoller.State.ALGAE_REVERSE),
                    Commands.waitUntil(m_clawRoller.stalled),
                    m_superStruct.getTransitionCommand(Arm.State.STOW, Elevator.State.STOW)));

        // Driver X Button: Send Arm and Elevator to LEVEL_2
        m_driver
            .x().and(isCoralMode)
            .onTrue(
                m_superStruct.getTransitionCommand(Arm.State.LEVEL_2, Elevator.State.LEVEL_2));

        // Driver X Button and Algae mode: Send Arm and Elevator to ALGAE_LOW position
        m_driver
            .x().and(isCoralMode.negate())
            .and(isProcessorMode.negate())
            .and(m_clawRoller.stalled.negate())
            .onTrue(
                Commands.sequence(
                    m_superStruct.getTransitionCommand(Arm.State.ALGAE_LOW,
                        Elevator.State.ALGAE_LOW),
                    m_clawRoller.setStateCommand(ClawRoller.State.ALGAE_FORWARD),
                    Commands.waitUntil(m_clawRoller.stalled),
                    m_superStruct.getTransitionCommand(Arm.State.STOW, Elevator.State.STOW)));

        m_driver
            .x().and(isCoralMode.negate())
            .and(isProcessorMode)
            .and(m_clawRoller.stalled.negate())
            .onTrue(
                Commands.sequence(
                    m_superStruct.getTransitionCommand(Arm.State.ALGAE_LOW_P,
                        Elevator.State.ALGAE_LOW_P),
                    m_clawRoller.setStateCommand(ClawRoller.State.ALGAE_REVERSE),
                    Commands.waitUntil(m_clawRoller.stalled),
                    m_superStruct.getTransitionCommand(Arm.State.STOW, Elevator.State.STOW)));

        // Driver B Button: Send Arm and Elevator to LEVEL_3
        m_driver
            .b().and(isCoralMode)
            .onTrue(
                m_superStruct.getTransitionCommand(Arm.State.LEVEL_3, Elevator.State.LEVEL_3));

        // Driver B Button and Algae mode: Send Arm and Elevator to ALGAE_HIGH position
        m_driver
            .b().and(isCoralMode.negate())
            .and(isProcessorMode.negate())
            .and(m_clawRoller.stalled.negate())
            .onTrue(
                Commands.sequence(
                    m_superStruct.getTransitionCommand(Arm.State.ALGAE_HIGH,
                        Elevator.State.ALGAE_HIGH),
                    m_clawRoller.setStateCommand(ClawRoller.State.ALGAE_FORWARD),
                    Commands.waitUntil(m_clawRoller.stalled),
                    m_superStruct.getTransitionCommand(Arm.State.STOW, Elevator.State.STOW)));

        m_driver
            .b().and(isCoralMode.negate())
            .and(isProcessorMode)
            .and(m_clawRoller.stalled.negate())
            .onTrue(
                Commands.sequence(
                    m_superStruct.getTransitionCommand(Arm.State.ALGAE_HIGH_P,
                        Elevator.State.ALGAE_HIGH_P),
                    m_clawRoller.setStateCommand(ClawRoller.State.ALGAE_REVERSE),
                    Commands.waitUntil(m_clawRoller.stalled),
                    m_superStruct.getTransitionCommand(Arm.State.STOW, Elevator.State.STOW)));

        // Driver Y Button: Send Arm and Elevator to LEVEL_4
        m_driver
            .y().and(isCoralMode)
            .onTrue(
                m_superStruct.getTransitionCommand(Arm.State.LEVEL_4, Elevator.State.LEVEL_4,
                    Units.degreesToRotations(10),
                    0.8));

        // Driver Y Button held and Right Bumper having been pressed to ALGAE mode: Send Arm and
        // Elevator to BARGE
        m_driver
            .y().and(isCoralMode.negate())
            .onTrue(
                m_superStruct.getTransitionCommand(Arm.State.BARGE, Elevator.State.BARGE));

        // Driver Right Trigger: Place Coral or Algae (Should be done once the robot is in position)
        m_driver.rightTrigger().and(isCoralMode)
            .whileTrue(
                m_clawRoller.setStateCommand(ClawRoller.State.SCORE))
            .onFalse(Commands.waitUntil(m_clawRollerLaserCAN.triggered.negate())
                .andThen(m_clawRoller.setStateCommand(ClawRoller.State.OFF))
                .andThen(m_superStruct.getTransitionCommand(Arm.State.STOW, Elevator.State.STOW)));

        // Score Algae
        m_driver.rightTrigger().and(isCoralMode.negate())
            .onTrue(Commands.either(m_clawRoller.setStateCommand(ClawRoller.State.ALGAE_FORWARD),
                m_clawRoller.setStateCommand(ClawRoller.State.ALGAE_REVERSE),
                () -> m_clawRoller.getState() == ClawRoller.State.ALGAE_REVERSE))
            .onFalse(Commands.waitUntil(m_clawRoller.stalled.negate())
                .andThen(Commands.waitSeconds(1))
                .andThen(m_clawRoller.setStateCommand(ClawRoller.State.OFF))
                .andThen(m_superStruct.getTransitionCommand(Arm.State.STOW, Elevator.State.STOW)));

        m_driver.leftTrigger().and(isCoralMode)
            .whileTrue(
                Commands.sequence(
                    m_clawRoller.setStateCommand(ClawRoller.State.INTAKE),
                    m_tounge.setStateCommand(Tounge.State.RAISED),
                    m_superStruct.getTransitionCommand(Arm.State.CORAL_INTAKE,
                        Elevator.State.CORAL_INTAKE, Units.degreesToRotations(10), .2),
                    Commands.waitUntil(
                        m_clawRollerLaserCAN.triggered
                            .and(m_tounge.coralContactTrigger)),
                    m_clawRoller.shuffleCommand(),
                    m_clawRoller.setStateCommand(ClawRoller.State.OFF)))
            .onFalse(
                Commands.sequence(
                    m_clawRoller.setStateCommand(ClawRoller.State.OFF),
                    m_superStruct.getTransitionCommand(Arm.State.STOW, Elevator.State.STOW),
                    m_tounge.lowerToungeCommand(),
                    m_driver.rumbleForTime(1, 1)));

        m_driver.back().onTrue(Commands.runOnce(() -> {
            m_profiledClimber.climbRequested = true;
            m_profiledClimber.climbStep += 1;
        }));

        m_profiledClimber.getClimbRequest().and(m_profiledClimber.getClimbStep1()).onTrue(
            m_profiledArm.setStateCommand(Arm.State.CLIMB)
                .andThen(m_profiledClimber.setStateCommand(Climber.State.PREP)));

        // Climb step 2: Move climber to climb
        m_profiledClimber.getClimbRequest().and(m_profiledClimber.getClimbStep2()).onTrue(
            m_profiledClimber.setStateCommand(Climber.State.CLIMB));

        m_profiledClimber.getClimbRequest().and(m_profiledClimber.getClimbStep3()).onTrue(
            m_profiledClimber.setStateCommand(Climber.State.ClIMB_MORE));


        m_driver.povLeft().onTrue(
            Commands.sequence(
                m_profiledElevator.setStateCommand(Elevator.State.STOW),
                m_tounge.setStateCommand(Tounge.State.DOWN),
                m_clawRoller.setStateCommand(State.SCORE)))
            .onFalse(m_clawRoller.setStateCommand(State.OFF)
                .andThen(m_tounge.setStateCommand(Tounge.State.STOW)));

        // Driver POV Right: Reset Climbing Sequence if needed
        m_driver
            .povRight()
            .onTrue(
                Commands.runOnce(
                    () -> {
                        m_profiledClimber.climbRequested = false;
                        m_profiledClimber.climbStep = 0;
                    }).andThen(m_profiledClimber.setStateCommand(Climber.State.HOME)).andThen(
                        m_superStruct.getTransitionCommand(Arm.State.STOW, Elevator.State.STOW)));

        // Slow drivetrain to 50% while climbing
        m_profiledClimber.getClimbRequest().whileTrue(
            DriveCommands.joystickDrive(
                m_drive,
                () -> -m_driver.getLeftY() * 0.5,
                () -> -m_driver.getLeftX() * 0.5,
                () -> -m_driver.getRightX() * 0.5));

        // Driver POV Down: Zero the Elevator (HOMING)
        m_driver.povDown().onTrue(m_profiledArm.setStateCommand(Arm.State.STOW)
            .andThen(m_profiledElevator.getHomeCommand()));

        // Toggles between horn and processor mode
        m_driver.povUp().onTrue(
            toggleProcessorMode()
                .andThen(m_driver.rumbleForTime(0.25, 1)));

        // Driver Right Bumper: Toggle between Coral and Algae Modes.
        // Make sure the Approach nearest reef face does not mess with this
        m_driver.start().and(m_driver.leftBumper().negate())
            .onTrue(setCoralAlgaeModeCommand()
                .andThen(m_superStruct.getTransitionCommand(Arm.State.STOW, Elevator.State.STOW))
                .andThen(m_clawRoller.setStateCommand(ClawRoller.State.OFF))
                .andThen(m_driver.rumbleForTime(0.25, 1)));

    }

    /**
     * Register Named commands for use in PathPlanner
     */
    private void registerNamedCommands()
    {
        // Go to the L4 Position
        NamedCommands.registerCommand(
            "L4",
            m_superStruct.getTransitionCommand(Arm.State.LEVEL_4, Elevator.State.LEVEL_4,
                Units.degreesToRotations(10),
                0.8));

        NamedCommands.registerCommand(
            "L4Prep",
            m_superStruct.getTransitionCommand(Arm.State.STOW, Elevator.State.LEVEL_4,
                Units.degreesToRotations(10),
                0.8));

        // Go to the Home Position
        NamedCommands.registerCommand(
            "Home",
            m_superStruct.getTransitionCommand(Arm.State.STOW, Elevator.State.STOW,
                Units.degreesToRotations(10), 0.8));

        // Intake Coral
        NamedCommands.registerCommand(
            "IntakeCoral",
            Commands.either(
                Commands.sequence(
                    m_clawRoller.setStateCommand(ClawRoller.State.INTAKE),
                    m_tounge.setStateCommand(Tounge.State.RAISED),
                    m_superStruct.getTransitionCommand(Arm.State.CORAL_INTAKE,
                        Elevator.State.CORAL_INTAKE, Units.degreesToRotations(10), .2),
                    Commands.waitUntil(
                        m_clawRollerLaserCAN.triggered
                            .and(m_tounge.coralContactTrigger)),
                    m_clawRoller.shuffleCommand(),
                    m_tounge.lowerToungeCommand()),
                m_tounge.lowerToungeCommand(),
                m_clawRollerLaserCAN.triggered.negate()));


        NamedCommands.registerCommand(
            "Score",
            Commands.sequence(
                m_clawRoller.setStateCommand(ClawRoller.State.SCORE),
                Commands.waitUntil(m_clawRollerLaserCAN.triggered.negate()),
                m_clawRoller.setStateCommand(ClawRoller.State.OFF)));
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

    public Command zeroTounge()
    {
        return m_tounge.zeroSensorCommand();
    }
}
