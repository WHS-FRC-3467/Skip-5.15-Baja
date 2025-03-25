package frc.robot.subsystems.GenericMotionProfiledSubsystem;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.util.drivers.CanDeviceId;
import frc.robot.util.sim.ArmSimConfiguration;
import frc.robot.util.sim.ElevatorSimConfiguration;
import frc.robot.util.sim.MotorSimConfiguration;

/** Wrapper class for TalonFX config params (Recommend initializing in a static block!) */
public class GenericMotionProfiledSubsystemConstants {

    public String kName = "ERROR_ASSIGN_A_NAME";

    public CanDeviceId kLeaderMotor = null;
    public CanDeviceId kFollowMotor = null;
    public CanDeviceId kCANcoder = null;

    public TalonFXConfiguration kMotorConfig = new TalonFXConfiguration();
    public TalonFXConfiguration kFollowerConfig = new TalonFXConfiguration();
    public TalonFXConfiguration kSimMotorConfig = new TalonFXConfiguration();
    public TalonFXConfiguration kSimFollowerConfig = new TalonFXConfiguration();
    public boolean kFollowerOpposesMain = false;

    public CANcoderConfiguration kEncoderConfig = new CANcoderConfiguration();

    // This is the minimum tolerance that will be used by the atPosition() method.
    // Subclasses should set their own values based on the mechanism they represent.
    public double kminTolerance = 0.00;

    public double kCANTimeout = 0.010; // use for important on the fly updates
    public int kLongCANTimeoutMs = 100; // use for constructors

    // Simulation Type
    public enum simType {
        ARM,
        ELEVATOR,
        ROLLER,
        NONE
    }

    public simType SimType = simType.NONE;

    // Motor simulation
    public MotorSimConfiguration kMotorSimConfig = new MotorSimConfiguration();

    // Arm Simulation
    public ArmSimConfiguration kArmSimConfig = new ArmSimConfiguration();

    // Elevator Simulation
    public ElevatorSimConfiguration kElevSimConfig = new ElevatorSimConfiguration();
}
