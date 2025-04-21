package frc.robot.commands;

import java.util.Map;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Elevator.Elevator;

/**
 * Command class for managing the robot's climbing process. Handles transitions between various
 * climb states based on the current climber status.
 */
public class ClimbCommand {
    private final Climber climber;
    private final Superstructure superstruct;

    // Internal step tracker to represent the current phase of the climb
    private int step = 0;

    /**
     * Private constructor to encapsulate setup logic.
     *
     * @param climber The climber subsystem.
     * @param superstruct The superstructure subsystem managing arm and elevator.
     */
    private ClimbCommand(Climber climber, Superstructure superstruct)
    {
        this.climber = climber;
        this.superstruct = superstruct;
    }

    /**
     * Determines the current climb step based on the climber's state.
     *
     * @return an integer representing the current phase of the climb process.
     */
    private int getClimbStep()
    {
        return switch (climber.getState()) {
            default -> 0; // Unknown or idle state
            case PREP -> 1; // Preparing for climb
            case CLIMB -> 2; // Actively climbing
            case MANUAL_CLIMB, HOLD -> 3; // Manual override or holding position
        };
    }

    /**
     * Creates a command that executes an appropriate climb behavior depending on the current climb
     * step.
     *
     * @return A state-driven command for climbing.
     */
    private Command get()
    {
        return Commands.select(
            // Maps step values to their respective commands
            Map.of(
                0, Commands.parallel(
                    // Step 0: Prep the climber and move arm/elevator to climb position
                    climber.setStateCommand(Climber.State.PREP),
                    superstruct.getDefaultTransitionCommand(Arm.State.CLIMB, Elevator.State.CLIMB)),
                1, climber.setStateCommand(Climber.State.CLIMB), // Step 1: Begin climbing
                2, climber.climbMore()), // Step 2: Manual override
            () -> step) // Selector function that chooses the command based on current step
            .beforeStarting(
                // Before starting, determine the current step using climber's state
                () -> step = getClimbStep());
    }

    /**
     * Public static method to create and return a climb command.
     *
     * @param climber The climber subsystem.
     * @param superstruct The superstructure subsystem.
     * @return A command that manages the climb sequence.
     */
    public static Command climbCommand(Climber climber, Superstructure superstruct)
    {
        return new ClimbCommand(climber, superstruct).get();
    }
}
