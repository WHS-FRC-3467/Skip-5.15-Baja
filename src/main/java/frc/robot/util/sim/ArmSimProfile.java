// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.sim;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.sim.PhysicsSim.SimProfile;
import frc.robot.util.sim.mechanisms.ArmElevComboMechanism;
import frc.robot.util.sim.mechanisms.MotionProfiledArmMechanism;
import frc.robot.util.sim.mechanisms.MotionProfiledMechanism;

/** Holds information about a simulated Single-Jointed Arm. */
public class ArmSimProfile extends SimProfile {

    private final String m_Name;
    private final TalonFX m_Talon;
    private final CANcoder m_CANcoder;
    private final MotorSimConfiguration m_MotorConst;
    private final ArmSimConfiguration m_ArmConst;
    private final SingleJointedArmSim m_ArmSim;
    private MotionProfiledMechanism m_Mech;

    /**
     * Creates a new simulation profile for a Single-Jointed Arm using the WPILib Arm sim.
     *
     * @param talon The TalonFX device
     * @param motorConst Motor Sim configuration values
     * @param armConst Arm Sim configuration values
     */
    public ArmSimProfile(
        final String simName,
        final TalonFX talon,
        final CANcoder cancoder,
        final MotorSimConfiguration motorConst,
        final ArmSimConfiguration armConst)
    {

        this.m_Name = simName;
        this.m_Talon = talon;
        this.m_CANcoder = cancoder;
        this.m_MotorConst = motorConst;
        this.m_ArmConst = armConst;

        DCMotor m_armGearbox = m_MotorConst.simMotorModelSupplier.get();

        // Create sim object
        this.m_ArmSim =
            new SingleJointedArmSim(
                m_armGearbox,
                m_ArmConst.kArmReduction,
                SingleJointedArmSim.estimateMOI(m_ArmConst.kArmLength, m_ArmConst.kArmMass),
                m_ArmConst.kArmLength,
                Units.degreesToRadians(m_ArmConst.kMinAngleDegrees),
                Units.degreesToRadians(m_ArmConst.kMaxAngleDegrees),
                true,
                Units.degreesToRadians(m_ArmConst.kDefaultArmSetpointDegrees));

        // Create sim mechanism
        if (m_ArmConst.kIsComboSim) {
            m_Mech = ArmElevComboMechanism.getInstance();
        } else {
            m_Mech = new MotionProfiledArmMechanism(m_Name);
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
        m_ArmSim.setInputVoltage(simState.getMotorVoltage());
        // Update the Arm Sim each time throuhgh the loop
        m_ArmSim.update(getPeriod());

        // Get current position and velocity of the Arm Sim ...
        final double position_rot = Units.radiansToRotations(m_ArmSim.getAngleRads());
        final double velocity_rps = Units.radiansToRotations(m_ArmSim.getVelocityRadPerSec());

        // ... and set the position and velocity for the lead motor simulation
        simState.setRawRotorPosition(position_rot * m_ArmConst.kArmReduction);
        simState.setRotorVelocity(velocity_rps * m_ArmConst.kArmReduction);

        // If using an external encoder, update its sim as well
        if (m_CANcoder != null) {
            m_CANcoder.getSimState().setRawPosition(position_rot * m_ArmConst.kSensorReduction);
            m_CANcoder.getSimState().setVelocity(velocity_rps * m_ArmConst.kSensorReduction);
        }

        // Update sim mechanism (in degrees)
        m_Mech.updateArm(Units.rotationsToDegrees(position_rot));
    }
}
