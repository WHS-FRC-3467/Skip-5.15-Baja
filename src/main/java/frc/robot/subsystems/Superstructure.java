// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Elevator.Elevator;

/**
 * Management class for synchronizing Arm and Elevator movements
 */
public class Superstructure {
    Arm arm;
    Elevator elevator;

    public Superstructure(Arm arm, Elevator elevator)
    {
        this.arm = arm;
        this.elevator = elevator;
    }

    /**
     * Get a Command to transition the states of the Arm and Elevator in the proper order.
     * 
     * This version allows you to pass individual tolerances for each mechanism.
     * 
     * (NOTE: Tolerances still have a floor as specified by the subsystem's kminTolerance
     * 
     * @param armState
     * @param elevatorState
     * @param armTolerance
     * @param elevTolerance
     * @param blocking Whether or not to wait for the elevator to raise
     * @return A Command
     */
    public Command getTransitionCommand(Arm.State armState, Elevator.State elevatorState,
        double armTolerance, double elevTolerance, boolean blocking)
    {
        return Commands.sequence(

            // Always move Arm to STOW position before moving Elevator
            Commands.either(
                Commands.none(), // If True
                Commands.sequence( // If False
                    arm.setStateCommand(Arm.State.STOW),
                    Commands.waitUntil(() -> arm.atPosition(armTolerance))),
                () -> arm.checkState(armState, arm.getState())), // Condition

            // Move Elevator to new position
            Commands.sequence(
                elevator.setStateCommand(elevatorState),
                Commands.waitUntil(() -> elevator.atPosition(elevTolerance))
                    .unless(() -> blocking)),

            // Reposition Arm to new position
            Commands.sequence(
                arm.setStateCommand(armState),
                Commands.waitUntil(() -> arm.atPosition(armTolerance))));
    }

    public Command getTransitionCommand(Arm.State armState, Elevator.State elevatorState,
        boolean blocking)
    {
        return getTransitionCommand(armState, elevatorState, 0.0,
            0.0, blocking).withName(
                "Superstructure Transition Command: Arm -> " + armState.name() +
                    " Elevator -> " + elevatorState.name());
    }

    /**
     * Get a Command to transition the states of the Arm and Elevator in the proper order.
     * 
     * This version uses default tolerance of 10deg and .4 rot
     * 
     * @param armState
     * @param elevatorState
     * @param blocking Whether or not to wait for the elevator to raise
     * @return Transition Command with normal tolerances
     */
    public Command getDefaultTransitionCommand(Arm.State armState, Elevator.State elevatorState,
        boolean blocking)
    {
        return getTransitionCommand(armState, elevatorState, Units.degreesToRotations(10),
            0.4, blocking).withName(
                "Superstructure Transition Command: Arm -> " + armState.name() +
                    " Elevator -> " + elevatorState.name());
    }

    public Command getDefaultTransitionCommand(Arm.State armState, Elevator.State elevatorState)
    {
        return getTransitionCommand(armState, elevatorState, Units.degreesToRotations(10),
            0.4, true).withName(
                "Superstructure Transition Command: Arm -> " + armState.name() +
                    " Elevator -> " + elevatorState.name());
    }

    /**
     * Get a Command to move the superstructure to the barging position, finishing at the elevator's
     * barge height to have velocity while spitting
     * 
     * @return A barge command
     */
    public Command bargeCommand()
    {
        return Commands.sequence(
            this.getDefaultTransitionCommand(Arm.State.STOW, Elevator.State.BARGE, false),
            Commands.waitUntil(elevator.launchHeightTrigger));
    }
}
