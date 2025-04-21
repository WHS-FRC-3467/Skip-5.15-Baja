package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.FieldConstants.ReefSide;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Claw.ClawRoller.ClawRoller;
import frc.robot.subsystems.Claw.ClawRoller.ClawRoller.State;
import frc.robot.subsystems.Claw.ClawRollerLaserCAN.ClawRollerLaserCAN;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Tongue.Tongue;
import frc.robot.subsystems.drive.Drive;

/**
 * Contains command generators for various autonomous and semi-autonomous algae scoring behaviors.
 * These include barge-style scoring, tongue spit maneuvers, and conditional strafing behaviors.
 */
public class ScoreCommands {
    // Singleton reference to RobotState for pose prediction and reef access
    private static final RobotState robotState = RobotState.getInstance();

    // Private constructor to prevent instantiation
    private ScoreCommands()
    {}

    /**
     * Creates a command that strafes the robot to align with the barge line, then actuates the
     * superstructure to score algae.
     *
     * @param drive The drive subsystem used to strafe.
     * @param robot A supplier for the current robot pose.
     * @param xSupplier Supplier of joystick X input for strafing control.
     * @param clawRoller The claw roller used to score the algae.
     * @param superstruct The superstructure to move arm and elevator into scoring configuration.
     * @return A complete command sequence for barge-style algae scoring.
     */
    public static Command bargeAlgae(Drive drive, Supplier<Pose2d> robot, DoubleSupplier xSupplier,
        ClawRoller clawRoller, Superstructure superstruct)
    {
        // Create a strafing command that aligns with the closest point on the barge line
        var strafeCommand = new JoystickStrafeCommand(
            drive,
            xSupplier,
            () -> robot.get().nearest(FieldConstants.Barge.bargeLine));

        // Run strafing and scoring sequence simultaneously
        return Commands.deadline(
            Commands.sequence(
                // Wait until robot is aligned close enough to barge line before scoring
                Commands.waitUntil(
                    () -> strafeCommand.withinTolerance(
                        Units.inchesToMeters(2.0),
                        Rotation2d.fromDegrees(4.0))),

                // Execute superstructure barge sequence (moves arm and elevator appropriately)
                superstruct.bargeCommand(),

                // Start scoring algae (reverse claw roller)
                clawRoller.setStateCommand(ClawRoller.State.ALGAE_REVERSE),

                // Wait until claw has fully ejected the algae (i.e., no longer "stopped")
                Commands.waitUntil(clawRoller.stopped.negate()),

                // Small delay for safety/stability
                Commands.waitSeconds(0.2),

                // Turn off the claw roller
                clawRoller.setStateCommand(ClawRoller.State.OFF)),

            // Cancel the strafe command when finished barging
            strafeCommand).finallyDo(interrupted -> {
                // Once finished and not interrupted, stow the arm and elevator
                if (!interrupted)
                    superstruct.getDefaultTransitionCommand(Arm.State.STOW,
                        Elevator.State.STOW).schedule();
            });
    }

    /**
     * Overloaded version of bargeAlgae that uses the drive subsystem's internal pose supplier.
     */
    public static Command bargeAlgae(Drive drive, DoubleSupplier xSupplier,
        ClawRoller clawRoller, Superstructure superstruct)
    {
        return bargeAlgae(drive, drive::getPose, xSupplier, clawRoller, superstruct);
    }

    /**
     * Creates a command that performs a spit and strafe maneuver based on the side of the reef.
     * This command ejects the game piece using the tongue and claw, then strafes to the side.
     *
     * @param drive The drive subsystem.
     * @param clawRoller The claw roller subsystem.
     * @param clawRollerLaserCAN The laser sensor used to detect algae presence.
     * @param tongue The tongue mechanism used for short-range algae ejection.
     * @param side The side of the reef (LEFT or RIGHT) to determine strafe direction.
     * @return A command that scores and then moves the robot laterally.
     */
    public static Command spitAndStrafe(Drive drive, ClawRoller clawRoller,
        ClawRollerLaserCAN clawRollerLaserCAN, Tongue tongue, ReefSide side)
    {
        return Commands.deadline(
            Commands.sequence(
                // Perform shuffle behavior to move algae into position
                clawRoller.L1ShuffleCommand(),

                // Extend tongue to L1 position for scoring
                tongue.setStateCommand(Tongue.State.L1),

                // Short delay before starting to spit
                Commands.waitSeconds(0.125),

                // Score the algae using the claw roller
                clawRoller.setStateCommand(ClawRoller.State.L1_SCORE),

                // Wait until algae is no longer detected by laser (algae has been scored)
                Commands.waitUntil(clawRollerLaserCAN.triggered.negate()),

                // Delay to ensure scoring is completed before movement
                Commands.waitSeconds(0.25)),

            // Select strafe direction based on reef side
            Commands.either(
                // RIGHT: strafe to the right (positive Y)
                new DriveToPose(
                    drive,
                    () -> robotState.getNearestReefBranch(side)
                        .transformBy(new Transform2d(
                            Constants.bumperWidth / 2 - Units.inchesToMeters(1),
                            Units.inchesToMeters(24),
                            Rotation2d.k180deg))),
                // LEFT: strafe to the left (negative Y)
                new DriveToPose(
                    drive,
                    () -> robotState.getNearestReefBranch(side)
                        .transformBy(new Transform2d(
                            Constants.bumperWidth / 2 - Units.inchesToMeters(1),
                            Units.inchesToMeters(-24),
                            Rotation2d.k180deg))),
                // Condition to choose which direction to strafe
                () -> side == ReefSide.RIGHT));
    }

    /**
     * Creates a command sequence to score a coral game piece using the claw roller and then stow
     * the superstructure.
     *
     * @param clawRoller The claw roller subsystem responsible for scoring the coral.
     * @param clawRollerLaserCAN The laser sensor used to detect the presence of coral.
     * @param superstruct The superstructure subsystem that manages the arm and elevator.
     * @return A command that scores the coral and stows the arm and elevator afterwards.
     */
    public static Command scoreCoral(ClawRoller clawRoller, ClawRollerLaserCAN clawRollerLaserCAN,
        Superstructure superstruct)
    {
        return Commands.sequence(
            // Start the claw roller in scoring mode
            clawRoller.setStateCommand(ClawRoller.State.SCORE),

            // Wait for coral to leave the claw (i.e., scoring is complete)
            Commands.either(
                // If the sensor has a valid measurement, wait until it no longer detects the coral,
                // then wait a short moment to ensure the coral is fully ejected
                Commands.sequence(
                    Commands.waitUntil(clawRollerLaserCAN.triggered.negate()),
                    Commands.waitSeconds(0.1)),

                // If the sensor is not valid, fallback to a fixed wait time
                Commands.waitSeconds(0.5),

                // Condition to determine which path to use: if the sensor measurement is valid
                clawRollerLaserCAN.validMeasurement),

            // Once scoring is complete, transition the arm and elevator to stowed position
            superstruct.getDefaultTransitionCommand(Arm.State.STOW, Elevator.State.STOW));
    }

    /**
     * Creates a command group to align to the nearest processor and bring the superstructure to the
     * relevant position
     *
     * @param drive The drive subsystem.
     * @param ySupplier Supplier of joystick Y input for approach control.
     * @param clawRoller The claw roller subsystem responsible for holding the algae.
     * @param superstruct The superstructure subsystem that manages the arm and elevator.
     * @return A command that aligns to the processor and brings the superstructure to the relevant
     *         position
     */
    public static Command processAlgae(Drive drive, DoubleSupplier ySupplier, ClawRoller clawRoller,
        Superstructure superstruct)
    {
        BooleanSupplier horns = () -> clawRoller.getState() == State.ALGAE_FORWARD;
        return Commands.parallel(
            Commands.either(
                superstruct.getDefaultTransitionCommand(Arm.State.PROCESSOR_SCORE,
                    Elevator.State.PROCESSOR_SCORE),
                superstruct.getDefaultTransitionCommand(Arm.State.STOW,
                    Elevator.State.PROCESSOR_SCORE),
                horns),
            DriveCommands.approachProcessor(drive, ySupplier));
    }
}
