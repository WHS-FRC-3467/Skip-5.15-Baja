package frc.robot.util.sim;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.sim.PhysicsSim.SimProfile;
import frc.robot.util.sim.mechanisms.GenericRollerSimMechanism;

/** Holds information about a simulated TalonFX. */
class RollerSimProfile extends SimProfile {

    private final String m_Name;
    private final TalonFX m_Talon;
    private final DCMotorSim m_MotorSim;
    private final MotorSimConfiguration m_MotorConst;
    private GenericRollerSimMechanism m_Mechanism = null;

    /**
     * Creates a new simulation profile for a roller system with provided DCMotorSim.
     *
     * @param talon The TalonFX device
     * @param motorModel A DCMotorSim
     */
    public RollerSimProfile(
        final String simName, final TalonFX talon, final MotorSimConfiguration motorConst)
    {

        this.m_Name = simName;
        this.m_Talon = talon;
        this.m_MotorConst = motorConst;

        this.m_MotorSim =
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    m_MotorConst.simMotorModelSupplier.get(),
                    m_MotorConst.simMOI,
                    m_MotorConst.simReduction),
                m_MotorConst.simMotorModelSupplier.get());

        this.m_Mechanism = new GenericRollerSimMechanism(m_Name, 1);
    }

    /**
     * Runs the simulation profile.
     *
     * <p>
     * This uses very rudimentary physics simulation and exists to allow users to test features of
     * our products in simulation using our examples out of the box. Users may modify this to
     * utilize more accurate physics simulation.
     */
    public void run()
    {

        // Get the simulation state for the lead motor
        var simState = m_Talon.getSimState();

        // set the supply (battery) voltage for the lead motor simulation state
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());

        // Set the input (voltage) to the Motor Simulation
        m_MotorSim.setInputVoltage(simState.getMotorVoltage());
        // Update the Motor Sim each time through the loop
        m_MotorSim.update(getPeriod());

        // Get current position and velocity of the Motor Sim ...
        final double position_rot = m_MotorSim.getAngularPositionRotations();
        final double velocity_rps =
            Units.radiansToRotations(m_MotorSim.getAngularVelocityRadPerSec());

        // ... and set the position and velocity for the lead motor simulation
        simState.setRawRotorPosition(position_rot);
        simState.setRotorVelocity(velocity_rps);

        // Update roller sim mechanism
        m_Mechanism.update(0, position_rot);
    }
}
