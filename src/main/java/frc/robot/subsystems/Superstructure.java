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
    Arm m_Arm;
    Elevator m_Elevator;

    public Superstructure(Arm arm, Elevator elevator)
    {
        m_Arm = arm;
        m_Elevator = elevator;
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
     * @return A Command
     */
    public Command getTransitionCommand(Arm.State armState, Elevator.State elevatorState,
        double armTolerance, double elevTolerance)
    {
        return Commands.sequence(

            // Always move Arm to STOW position before moving Elevator
            Commands.either(
                Commands.none(), // If True
                Commands.sequence( // If False
                    m_Arm.setStateCommand(Arm.State.STOW),
                    Commands.waitUntil(() -> m_Arm.atPosition(armTolerance))),
                () -> m_Arm.checkState(armState, m_Arm.getState())), // Condition

            // Move Elevator to new position
            Commands.sequence(
                m_Elevator.setStateCommand(elevatorState),
                Commands.waitUntil(() -> m_Elevator.atPosition(elevTolerance))),

            // Reposition Arm to new position
            Commands.sequence(
                m_Arm.setStateCommand(armState),
                Commands.waitUntil(() -> m_Arm.atPosition(armTolerance))));
    }

    public Command getTransitionCommand(Arm.State armState, Elevator.State elevatorState)
    {
        return getTransitionCommand(armState, elevatorState, 0.0,
            0.0).withName(
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
     * @return Transtition Command with normal tolerances
     */
    public Command getDefaultTransitionCommand(Arm.State armState, Elevator.State elevatorState)
    {
        return getTransitionCommand(armState, elevatorState, Units.degreesToRotations(10),
            0.4).withName(
                "Superstructure Transition Command: Arm -> " + armState.name() +
                    " Elevator -> " + elevatorState.name());
    }
}
