package frc.robot.subsystems.Hopper.Hopper;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Ports;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemConstants;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemConstants.simType;

/** Add your docs here. */
public final class HopperConstants {

    public static final GenericMotionProfiledSubsystemConstants kSubSysConstants =
        new GenericMotionProfiledSubsystemConstants();

    static {
        kSubSysConstants.kName = "Hopper";

        kSubSysConstants.kLeaderMotor = Ports.HOPPER;
        kSubSysConstants.kFollowMotor = null;
        kSubSysConstants.kFollowerOpposesMain = true;

        // Using TalonFX internal encoder
        kSubSysConstants.kCANcoder = null;
        kSubSysConstants.kMotorConfig.Feedback.FeedbackSensorSource =
            FeedbackSensorSourceValue.RotorSensor;
        kSubSysConstants.kMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
        kSubSysConstants.kMotorConfig.Feedback.RotorToSensorRatio = 1.0;


        kSubSysConstants.kMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kSubSysConstants.kMotorConfig.MotorOutput.Inverted =
            InvertedValue.CounterClockwise_Positive;

        kSubSysConstants.kMotorConfig.Voltage.PeakForwardVoltage = 12.0;
        kSubSysConstants.kMotorConfig.Voltage.PeakReverseVoltage = -12.0;

        kSubSysConstants.kMotorConfig.CurrentLimits.SupplyCurrentLimit = 30;
        kSubSysConstants.kMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        kSubSysConstants.kMotorConfig.CurrentLimits.StatorCurrentLimit = 55;
        kSubSysConstants.kMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        kSubSysConstants.kMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
        kSubSysConstants.kMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        kSubSysConstants.kMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        kSubSysConstants.kMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        /* REAL system profile constants */
        kSubSysConstants.kMotorConfig.Slot0.kP = 1;
        kSubSysConstants.kMotorConfig.Slot0.kI = 0;
        kSubSysConstants.kMotorConfig.Slot0.kD = 0;
        kSubSysConstants.kMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        kSubSysConstants.kMotorConfig.Slot0.kG = 0;
        kSubSysConstants.kMotorConfig.Slot0.kS = 0;
        kSubSysConstants.kMotorConfig.Slot0.kV = 0;
        kSubSysConstants.kMotorConfig.Slot0.kA = 0;
        kSubSysConstants.kMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 1;
        kSubSysConstants.kMotorConfig.MotionMagic.MotionMagicAcceleration = 1;
        kSubSysConstants.kMotorConfig.MotionMagic.MotionMagicJerk = 0;

        /* SIM system profile constants */
        kSubSysConstants.kSimMotorConfig.Slot0.kP = 1;
        kSubSysConstants.kSimMotorConfig.Slot0.kI = 0;
        kSubSysConstants.kSimMotorConfig.Slot0.kD = 0;
        kSubSysConstants.kSimMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        kSubSysConstants.kSimMotorConfig.Slot0.kG = 0;
        kSubSysConstants.kSimMotorConfig.Slot0.kS = 0;
        kSubSysConstants.kSimMotorConfig.Slot0.kV = 0;
        kSubSysConstants.kSimMotorConfig.Slot0.kA = 0;
        kSubSysConstants.kSimMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 1;
        kSubSysConstants.kSimMotorConfig.MotionMagic.MotionMagicAcceleration = 1;
        kSubSysConstants.kSimMotorConfig.MotionMagic.MotionMagicJerk = 0;

        // Simulation Type
        kSubSysConstants.SimType = simType.ROLLER;

        // Motor simulation
        kSubSysConstants.kMotorSimConfig.simMotorModelSupplier = () -> DCMotor.getKrakenX60Foc(1);
        kSubSysConstants.kMotorSimConfig.simReduction = 1;
    }
}
