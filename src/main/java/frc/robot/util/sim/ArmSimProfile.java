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

    private final TalonFX talon;
    private final CANcoder CANCoder;
    private final ArmSimConfiguration armConst;
    private final SingleJointedArmSim armSim;
    private MotionProfiledMechanism mech;

    /**
     * Creates a new simulation profile for a Single-Jointed Arm using the WPILib Arm sim.
     *
     * @param talon The TalonFX device
     * @param motorConst Motor Sim configuration values
     * @param armConst Arm Sim configuration values
     */
    public ArmSimProfile(
        final String name,
        final TalonFX talon,
        final CANcoder CANcoder,
        final MotorSimConfiguration motorConst,
        final ArmSimConfiguration armConst)
    {

        this.talon = talon;
        this.CANCoder = CANcoder;
        this.armConst = armConst;

        DCMotor armGearbox = motorConst.simMotorModelSupplier.get();

        // Create sim object
        this.armSim =
            new SingleJointedArmSim(
                armGearbox,
                armConst.kArmReduction,
                SingleJointedArmSim.estimateMOI(armConst.kArmLength, armConst.kArmMass),
                armConst.kArmLength,
                Units.degreesToRadians(armConst.kMinAngleDegrees),
                Units.degreesToRadians(armConst.kMaxAngleDegrees),
                true,
                Units.degreesToRadians(armConst.kDefaultArmSetpointDegrees));

        // Create sim mechanism
        if (armConst.kIsComboSim) {
            mech = ArmElevComboMechanism.getInstance();
        } else {
            mech = new MotionProfiledArmMechanism(name);
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
        armSim.setInputVoltage(simState.getMotorVoltage());
        // Update the Arm Sim each time throuhgh the loop
        armSim.update(getPeriod());

        // Get current position and velocity of the Arm Sim ...
        final double position_rot = Units.radiansToRotations(armSim.getAngleRads());
        final double velocity_rps = Units.radiansToRotations(armSim.getVelocityRadPerSec());

        // ... and set the position and velocity for the lead motor simulation
        simState.setRawRotorPosition(position_rot * armConst.kArmReduction);
        simState.setRotorVelocity(velocity_rps * armConst.kArmReduction);

        // If using an external encoder, update its sim as well
        if (CANCoder != null) {
            CANCoder.getSimState().setRawPosition(position_rot * armConst.kSensorReduction);
            CANCoder.getSimState().setVelocity(velocity_rps * armConst.kSensorReduction);
        }

        // Update sim mechanism (in degrees)
        mech.updateArm(Units.rotationsToDegrees(position_rot));
    }
}
