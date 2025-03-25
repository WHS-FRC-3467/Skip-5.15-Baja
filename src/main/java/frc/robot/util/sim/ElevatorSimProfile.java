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

    private final String m_Name;
    private final TalonFX m_Talon;
    private final CANcoder m_CANcoder;
    private final MotorSimConfiguration m_MotorConst;
    private final ElevatorSimConfiguration m_ElevConst;
    private final ElevatorSim m_ElevatorSim;
    private final MotionProfiledMechanism m_Mech;

    /**
     * Creates a new simulation profile for an Elevator using the WPILib Elevator sim.
     *
     * @param talon The TalonFX device
     * @param motorConst Motor Sim configuration values
     * @param armConst Arm Sim configuration values
     */
    public ElevatorSimProfile(
        final String simName,
        final TalonFX talon,
        final CANcoder cancoder,
        final MotorSimConfiguration motorConst,
        final ElevatorSimConfiguration elevConst)
    {
        this.m_Name = simName;
        this.m_Talon = talon;
        this.m_CANcoder = cancoder;
        this.m_MotorConst = motorConst;
        this.m_ElevConst = elevConst;

        DCMotor elevatorGearbox = m_MotorConst.simMotorModelSupplier.get();

        // Create sim object
        this.m_ElevatorSim =
            new ElevatorSim(
                elevatorGearbox,
                m_ElevConst.kElevatorGearing,
                m_ElevConst.kCarriageMass,
                m_ElevConst.kElevatorDrumRadius,
                m_ElevConst.kMinElevatorHeight,
                m_ElevConst.kMaxElevatorHeight,
                true,
                m_ElevConst.kDefaultSetpoint);

        // Create sim mechanism
        if (m_ElevConst.kIsComboSim) {
            m_Mech = ArmElevComboMechanism.getInstance();
        } else {
            m_Mech = new MotionProfiledElevatorMechanism(m_Name);
        }
    }

    /** Runs the simulation profile. */
    public void run()
    {

        // Get the simulation state for the lead motor
        var simState = m_Talon.getSimState();

        // set the supply (battery) voltage for the lead motor simulation state
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());

        // Set the input (voltage) to the Arm Simulation
        m_ElevatorSim.setInputVoltage(simState.getMotorVoltage());
        // Update the Elevator Sim each time throuhgh the loop
        m_ElevatorSim.update(getPeriod());

        // Get current position and velocity of the Elevator Sim ...
        double position_meters = m_ElevatorSim.getPositionMeters();
        double velocity_mps = m_ElevatorSim.getVelocityMetersPerSecond();

        // ... and set the position and velocity for the lead motor simulation
        // (Multiply elevator positon by total gearing reduction from motor to elevator)
        simState.setRawRotorPosition(position_meters
            / (2 * Math.PI * m_ElevConst.kElevatorDrumRadius) * m_ElevConst.kElevatorGearing);
        simState.setRotorVelocity(velocity_mps / (2 * Math.PI * m_ElevConst.kElevatorDrumRadius)
            * m_ElevConst.kElevatorGearing);

        // If using an external encoder, update its sim as well
        if (m_CANcoder != null) {
            // (Multiply elevator position by gearing reduction from sensor to elevator)
            m_CANcoder.getSimState().setRawPosition(position_meters
                / (2 * Math.PI * m_ElevConst.kElevatorDrumRadius) * m_ElevConst.kSensorReduction);
            m_CANcoder.getSimState().setVelocity(velocity_mps
                / (2 * Math.PI * m_ElevConst.kElevatorDrumRadius) * m_ElevConst.kSensorReduction);
        }

        // Update elevator sim mechanism
        m_Mech.updateElevator(position_meters);
    }
}
