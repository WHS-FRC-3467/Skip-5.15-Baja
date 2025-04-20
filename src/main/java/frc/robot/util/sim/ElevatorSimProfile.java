// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.sim;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.util.sim.PhysicsSim.SimProfile;
import frc.robot.util.sim.mechanisms.ArmElevComboMechanism;
import frc.robot.util.sim.mechanisms.MotionProfiledElevatorMechanism;
import frc.robot.util.sim.mechanisms.MotionProfiledMechanism;

/** Holds information about a simulated Elevatore. */
public class ElevatorSimProfile extends SimProfile {

    private final TalonFX talon;
    private final CANcoder CANCoder;
    private final ElevatorSimConfiguration elevConst;
    private final ElevatorSim elevatorSim;
    private final MotionProfiledMechanism mech;

    /**
     * Creates a new simulation profile for an Elevator using the WPILib Elevator sim.
     *
     * @param talon The TalonFX device
     * @param motorConst Motor Sim configuration values
     * @param armConst Arm Sim configuration values
     */
    public ElevatorSimProfile(
        final String name,
        final TalonFX talon,
        final CANcoder CANCoder,
        final MotorSimConfiguration motorConst,
        final ElevatorSimConfiguration elevConst)
    {
        this.talon = talon;
        this.CANCoder = CANCoder;
        this.elevConst = elevConst;

        DCMotor elevatorGearbox = motorConst.simMotorModelSupplier.get();

        // Create sim object
        this.elevatorSim =
            new ElevatorSim(
                elevatorGearbox,
                elevConst.kElevatorGearing,
                elevConst.kCarriageMass,
                elevConst.kElevatorDrumRadius,
                elevConst.kMinElevatorHeight,
                elevConst.kMaxElevatorHeight,
                true,
                elevConst.kDefaultSetpoint);

        // Create sim mechanism
        if (elevConst.kIsComboSim) {
            mech = ArmElevComboMechanism.getInstance();
        } else {
            mech = new MotionProfiledElevatorMechanism(name);
        }
    }

    /** Runs the simulation profile. */
    public void run()
    {

        // Get the simulation state for the lead motor
        var simState = talon.getSimState();

        // set the supply (battery) voltage for the lead motor simulation state
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());

        // Set the input (voltage) to the Arm Simulation
        elevatorSim.setInputVoltage(simState.getMotorVoltage());
        // Update the Elevator Sim each time throuhgh the loop
        elevatorSim.update(getPeriod());

        // Get current position and velocity of the Elevator Sim ...
        double position_meters = elevatorSim.getPositionMeters();
        double velocity_mps = elevatorSim.getVelocityMetersPerSecond();

        // ... and set the position and velocity for the lead motor simulation
        // (Multiply elevator positon by total gearing reduction from motor to elevator)
        simState.setRawRotorPosition(position_meters
            / (2 * Math.PI * elevConst.kElevatorDrumRadius) * elevConst.kElevatorGearing);
        simState.setRotorVelocity(velocity_mps / (2 * Math.PI * elevConst.kElevatorDrumRadius)
            * elevConst.kElevatorGearing);

        // If using an external encoder, update its sim as well
        if (CANCoder != null) {
            // (Multiply elevator position by gearing reduction from sensor to elevator)
            CANCoder.getSimState().setRawPosition(position_meters
                / (2 * Math.PI * elevConst.kElevatorDrumRadius) * elevConst.kSensorReduction);
            CANCoder.getSimState().setVelocity(velocity_mps
                / (2 * Math.PI * elevConst.kElevatorDrumRadius) * elevConst.kSensorReduction);
        }

        // Update elevator sim mechanism
        mech.updateElevator(position_meters);
    }
}
