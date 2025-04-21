// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.Vision.VisionConstants.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.RobotType;
import frc.robot.FieldConstants.ReefSide;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.JoystickApproachCommand;
import frc.robot.commands.ScoreCommands;
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
 * Container class for the entire robot structure. Handles subsystem instantiations, input devices,
 * and command bindings.
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
    private final Superstructure superstruct;

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

    /** Constructs the RobotContainer. Initializes all hardware and simulation IO. */
    public RobotContainer()
    {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, use hardware IOs
                drive = new Drive(
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
                vision = new Vision(
                    drive,
                    new VisionIOPhotonVision(camera0Name, robotToCamera0),
                    new VisionIOPhotonVision(camera1Name, robotToCamera1));

                LED = Constants.getRobot() == RobotType.BAJA
                    ? new LEDSubsystem(new LEDSubsystemIOCANdle(), clawRoller, arm, elevator,
                        climber, vision, clawRollerLaserCAN.triggered, isCoralMode)
                    : null;

                hasVision = new Trigger(() -> vision.anyCameraConnected);
                break;

            case SIM:
                // Simulated robot
                drive = new Drive(
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
                LED = new LEDSubsystem(new LEDSubsystemIOWPILib(), clawRoller, arm, elevator,
                    climber, vision, clawRollerLaserCAN.triggered, isCoralMode);

                hasVision = new Trigger(() -> true);
                break;

            default:
                // Replayed logs or fallback mode
                drive = new Drive(
                    new GyroIO() {},
                    new ModuleIO() {}, new ModuleIO() {},
                    new ModuleIO() {}, new ModuleIO() {});

                arm = new Arm(new ArmIO() {}, true);
                elevator = new Elevator(new ElevatorIOSim(), true);
                climber = new Climber(new ClimberIO() {}, true);
                clawRoller = new ClawRoller(new ClawRollerIO() {}, true);
                tongue = new Tongue(new TongueIO() {}, true);
                clawRollerLaserCAN = new ClawRollerLaserCAN(new ClawRollerLaserCANIO() {});
                isCoralMode = new Trigger(clawRollerLaserCAN.triggered.debounce(0.25));

                vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
                LED = new LEDSubsystem(new LEDSubsystemIO() {}, clawRoller, arm, elevator, climber,
                    vision, clawRollerLaserCAN.triggered, isCoralMode);

                hasVision = new Trigger(() -> vision.anyCameraConnected);
                break;
        }

        hasLaserCAN = new Trigger(clawRollerLaserCAN.validMeasurement);
        superstruct = new Superstructure(arm, elevator);
        configureControllerBindings();

        DriverStation.silenceJoystickConnectionWarning(false);
    }

    /** @return Default alliance-relative driving command using joystick input. */
    private Command joystickDrive()
    {
        return DriveCommands.joystickDrive(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX());
    }

    /** @return Command to approach the reef branch on the given side using joystick Y input. */
    private JoystickApproachCommand approachReefBranch(ReefSide side)
    {
        return DriveCommands.approachReefBranch(drive, () -> -driver.getLeftY(), side,
            alignPredictionSeconds);
    }

    /** @return Command to turn the robot to face the nearest reef face. */
    private Command faceReef()
    {
        return DriveCommands.faceReef(drive, () -> -driver.getLeftX(), () -> -driver.getLeftY(),
            alignPredictionSeconds);
    }

    /** @return Intake coral using roller, laser, tongue, and superstructure. */
    private Command intakeCoral()
    {
        return IntakeCommands.intakeCoral(clawRoller, clawRollerLaserCAN, tongue, superstruct,
            driver);
    }

    /** @return Descore algae into the reef horns. */
    private Command descoreAlgae(boolean horns)
    {
        return IntakeCommands.descoreAlgae(drive, alignPredictionSeconds, () -> -driver.getLeftY(),
            clawRoller, superstruct, horns);
    }

    /** @return Collect algae using lollipop method. */
    private Command lolipopAlgae()
    {
        return IntakeCommands.lolipopAlgae(drive, alignPredictionSeconds, () -> -driver.getLeftY(),
            clawRoller, superstruct);
    }

    /** @return Ground intake for algae. */
    private Command groundAlgae()
    {
        return IntakeCommands.groundAlgae(clawRoller, superstruct);
    }

    /** @return Spit and strafe away from reef after scoring. */
    private Command spitAndStrafe(ReefSide side)
    {
        return ScoreCommands.spitAndStrafe(drive, clawRoller, clawRollerLaserCAN, tongue, side);
    }

    /** @return Score algae by barging into the reef. */
    private Command bargeAlgae()
    {
        return ScoreCommands.bargeAlgae(drive, () -> -driver.getLeftX(), clawRoller, superstruct);
    }

    /** @return Score coral using sensor and command logic. */
    private Command scoreCoral()
    {
        return ScoreCommands.scoreCoral(clawRoller, clawRollerLaserCAN, superstruct);
    }

    /** @return Command to score algae into processor. */
    private Command processAlgae()
    {
        return ScoreCommands.processAlgae(drive, () -> -driver.getLeftY(), clawRoller, superstruct);
    }

    /** @return Composite climbing command. */
    private Command climbCommand()
    {
        return ClimbCommand.climbCommand(climber, superstruct);
    }

    /** @return Toggle command for reversing algae intake direction. */
    private Command reverseAlgaeDirection()
    {
        return Commands.either(
            clawRoller.setStateCommand(ClawRoller.State.ALGAE_FORWARD),
            clawRoller.setStateCommand(ClawRoller.State.ALGAE_REVERSE),
            () -> clawRoller.getState() == ClawRoller.State.ALGAE_REVERSE);
    }

    /** Binds buttons to corresponding robot commands. */
    private void configureControllerBindings()
    {
        drive.setDefaultCommand(joystickDrive());

        driver.rightBumper().and(hasVision).whileTrue(
            Commands.either(approachReefBranch(ReefSide.RIGHT), Commands.none(), isCoralMode));

        driver.leftBumper().and(hasVision).whileTrue(
            Commands.either(approachReefBranch(ReefSide.LEFT), Commands.none(), isCoralMode));

        driver.leftBumper().and(driver.rightBumper()).and(isCoralMode.negate())
            .whileTrue(descoreAlgae(true));

        driver.leftBumper().and(driver.a()).whileTrue(spitAndStrafe(ReefSide.LEFT));
        driver.rightBumper().and(driver.a()).whileTrue(spitAndStrafe(ReefSide.RIGHT));

        driver.a().onTrue(
            Commands.either(
                superstruct.getDefaultTransitionCommand(Arm.State.LEVEL_1, Elevator.State.LEVEL_1),
                groundAlgae(),
                isCoralMode))
            .whileTrue(
                Commands.either(faceReef(), Commands.none(), isCoralMode));

        driver.start().whileTrue(descoreAlgae(false));

        driver.x().and(isCoralMode)
            .onTrue(
                superstruct.getDefaultTransitionCommand(Arm.State.LEVEL_2, Elevator.State.LEVEL_2));

        driver.x().and(isCoralMode.negate())
            .onTrue(lolipopAlgae());

        driver.b().and(isCoralMode)
            .onTrue(
                superstruct.getDefaultTransitionCommand(Arm.State.LEVEL_3, Elevator.State.LEVEL_3));

        driver.b().and(isCoralMode.negate())
            .whileTrue(processAlgae());

        driver.y().and(isCoralMode)
            .onTrue(
                superstruct.getDefaultTransitionCommand(Arm.State.LEVEL_4, Elevator.State.LEVEL_4));

        driver.y().and(isCoralMode.negate())
            .onTrue(bargeAlgae());

        driver.rightTrigger().and(driver.a().negate()).onTrue(
            Commands.either(scoreCoral(), reverseAlgaeDirection(), isCoralMode));

        driver.leftTrigger().whileTrue(intakeCoral());
        driver.back().whileTrue(climbCommand());

        driver.povLeft().onTrue(
            Commands.sequence(
                elevator.setStateCommand(Elevator.State.STOW),
                tongue.setStateCommand(Tongue.State.DOWN),
                clawRoller.setStateCommand(State.SCORE)))
            .onFalse(clawRoller.setStateCommand(State.OFF)
                .andThen(tongue.setStateCommand(Tongue.State.STOW)));

        driver.povDown().onTrue(
            Commands.sequence(
                arm.setStateCommand(Arm.State.STOW),
                elevator.getHomeCommand()));

        driver.povUp().onTrue(
            Commands.parallel(
                elevator.setStateCommand(Elevator.State.LEVEL_3),
                arm.setStateCommand(Arm.State.LEVEL_2)));
    }

    /**
     * Returns the autonomous command to run during the autonomous period.
     * 
     * @return Autonomous command (currently none)
     */
    public Command getAutonomousCommand()
    {
        return Commands.none();
    }
}
