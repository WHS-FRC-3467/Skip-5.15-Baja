package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Claw.ClawRoller.ClawRoller;
import frc.robot.subsystems.Claw.ClawRollerLaserCAN.ClawRollerLaserCAN;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Tongue.Tongue;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.util.WindupXboxController;

/**
 * A utility class that defines reusable command sequences for handling algae game piece intake,
 * scoring, and descoring actions using the robot’s subsystems.
 */
public class IntakeCommands {
    // Singleton instance of RobotState to access prediction and reef face positioning
    private static final RobotState robotState = RobotState.getInstance();

    // Private constructor to prevent instantiation
    private IntakeCommands()
    {}

    public static Command intakeCoral(ClawRoller clawRoller, ClawRollerLaserCAN clawRollerLaserCAN,
        Tongue tongue, Superstructure superstruct, WindupXboxController controller)
    {
        return Commands.sequence(
            tongue.setStateCommand(Tongue.State.RAISED),
            superstruct.getDefaultTransitionCommand(Arm.State.CORAL_INTAKE,
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
                clawRollerLaserCAN.validMeasurement), // If lasercan is not valid, don't check it
                                                      // while intaking
            clawRoller.shuffleCommand(),
            clawRoller.setStateCommand(ClawRoller.State.HOLDCORAL))
            .finallyDo(() -> Commands.sequence(
                clawRoller.setStateCommand(ClawRoller.State.OFF),
                superstruct.getDefaultTransitionCommand(Arm.State.STOW, Elevator.State.STOW),
                tongue.lowerTongueCommand(),
                controller.rumbleForTime(0.25, 1)).schedule());
    }

    /**
     * Creates a command to drive toward a reef face and descore algae using the claw roller.
     *
     * @param drive The drive subsystem for approaching the reef face.
     * @param predictionSeconds Seconds into the future to predict the reef face location.
     * @param ySupplier Supplier for joystick Y-axis input used to control approach.
     * @param clawRoller The claw roller subsystem to control algae movement.
     * @param superstruct The superstructure for coordinating arm and elevator transitions.
     * @param horns A supplier that determines whether to descore to the horns or main claw.
     * @return A command that approaches, descends, and descoring algae autonomously.
     */
    public static Command descoreAlgae(Drive drive, DoubleSupplier predictionSeconds,
        DoubleSupplier ySupplier, ClawRoller clawRoller, Superstructure superstruct,
        boolean horns)
    {
        // Command to approach the reef face using joystick input
        var approachCommand = new JoystickApproachCommand(
            drive,
            ySupplier,
            () -> robotState.getNearestReefFace(predictionSeconds.getAsDouble()));

        // Sequence of actions to run while approaching the target
        return Commands.deadline(
            Commands.sequence(
                // Start algae outtake with the claw
                clawRoller.setStateCommand(
                    horns ? ClawRoller.State.ALGAE_FORWARD : ClawRoller.State.ALGAE_REVERSE),

                // Choose superstructure state based on horns mode and the position of the algae on
                // the current reef face
                horns ? Commands.either(
                    superstruct.getDefaultTransitionCommand(Arm.State.ALGAE_HIGH,
                        Elevator.State.ALGAE_HIGH),
                    superstruct.getDefaultTransitionCommand(Arm.State.ALGAE_LOW,
                        Elevator.State.ALGAE_LOW),
                    () -> robotState.isAlgaeHigh(predictionSeconds.getAsDouble()))
                    : Commands.either(
                        superstruct.getDefaultTransitionCommand(Arm.State.ALGAE_HIGH_P,
                            Elevator.State.ALGAE_HIGH_P),
                        superstruct.getDefaultTransitionCommand(Arm.State.ALGAE_LOW_P,
                            Elevator.State.ALGAE_LOW_P),
                        () -> robotState.isAlgaeHigh(predictionSeconds.getAsDouble())),

                // Wait until the claw roller stalls (indicating the algae has been descored)
                Commands.waitUntil(clawRoller.stalled)),
            // Cancel the approach command when finished descoring
            approachCommand)
            // Once complete and not interrupted, return to the stowed position (non-blocking)
            .finallyDo(interrupted -> {
                if (!interrupted) {
                    superstruct.getDefaultTransitionCommand(
                        Arm.State.STOW,
                        Elevator.State.ALGAE_STOW).schedule();
                }
            });
    }

    /**
     * Overload of descoreAlgae that assumes no prediction offset.
     */
    public static Command descoreAlgae(Drive drive, DoubleSupplier ySupplier, ClawRoller clawRoller,
        Superstructure superstruct, boolean horns)
    {
        return descoreAlgae(drive, () -> 0.0, ySupplier, clawRoller, superstruct, horns);
    }

    /**
     * Creates a command to drive toward a lolipop and descore algae using the claw roller.
     *
     * @param drive The drive subsystem for approaching the reef face.
     * @param predictionSeconds Seconds into the future to predict the lolipop location.
     * @param ySupplier Supplier for joystick Y-axis input used to control approach.
     * @param clawRoller The claw roller subsystem.
     * @param superstruct The superstructure for coordinating arm and elevator transitions.
     * @return A command that approaches, descends, and descoring algae autonomously.
     */
    public static Command lolipopAlgae(Drive drive, DoubleSupplier predictionSeconds,
        DoubleSupplier ySupplier, ClawRoller clawRoller, Superstructure superstruct)
    {
        // Command to approach the reef face using joystick input
        var approachCommand = new JoystickApproachCommand(
            drive,
            () -> ySupplier.getAsDouble() * 0.85, // Drive slower during lolipop collect
            () -> robotState.getNearestLolipop(predictionSeconds.getAsDouble()));

        // Sequence of actions to run while approaching the target
        return Commands.deadline(
            Commands.sequence(
                // Start algae outtake with the claw
                clawRoller.setStateCommand(ClawRoller.State.ALGAE_FORWARD),

                // Bring superstructure to lolipop position
                superstruct.getDefaultTransitionCommand(Arm.State.PROCESSOR_SCORE,
                    Elevator.State.ALGAE_LOLLIPOP),

                // Wait until the claw roller stalls (indicating the algae has been descored)
                Commands.waitUntil(clawRoller.stalled)),

            // Cancel the approach command when finished descoring
            approachCommand)
            // Once complete and not interrupted, return to the stowed position (non-blocking)
            .finallyDo(interrupted -> {
                if (!interrupted) {
                    superstruct.getDefaultTransitionCommand(
                        Arm.State.STOW,
                        Elevator.State.ALGAE_STOW).schedule();
                }
            });
    }

    /**
     * Overload of
     * {@link #lolipopAlgae(Drive, DoubleSupplier, DoubleSupplier, ClawRoller, Superstructure)} with
     * zero prediction time.
     *
     * @param drive The drive subsystem.
     * @param ySupplier Supplies the joystick Y input.
     * @param clawRoller The claw roller subsystem.
     * @param superstruct The superstructure for coordinating arm and elevator transitions.
     * @return A command that drives the robot with joystick input while rotating it to face the
     *         reef.
     */
    public static Command lolipopAlgae(Drive drive, DoubleSupplier ySupplier, ClawRoller clawRoller,
        Superstructure superstruct)
    {
        return lolipopAlgae(drive, () -> 0.0, ySupplier, clawRoller, superstruct);
    }

    /**
     * Creates a command sequence to collect algae from the ground.
     *
     * @param clawRoller The claw roller to reverse and intake the algae.
     * @param superstruct The superstructure for transitioning to ground pickup positions.
     * @return A command that moves the robot to ground pickup configuration, reverses the claw to
     *         intake algae, and returns to stowed.
     */
    public static Command groundAlgae(ClawRoller clawRoller, Superstructure superstruct)
    {
        return Commands.sequence(
            // Move arm and elevator to ground intake position
            superstruct.getDefaultTransitionCommand(Arm.State.ALGAE_GROUND,
                Elevator.State.CORAL_INTAKE),

            // Activate claw roller to pull algae in
            clawRoller.setStateCommand(ClawRoller.State.ALGAE_REVERSE),

            // Wait until algae is fully collected (roller stalls)
            Commands.waitUntil(clawRoller.stalled))

            // Once complete and not interrupted, return to the stowed position (non-blocking)
            .finallyDo(interrupted -> {
                if (!interrupted) {
                    superstruct.getDefaultTransitionCommand(
                        Arm.State.STOW,
                        Elevator.State.ALGAE_STOW).schedule();
                }
            });
    }
}
