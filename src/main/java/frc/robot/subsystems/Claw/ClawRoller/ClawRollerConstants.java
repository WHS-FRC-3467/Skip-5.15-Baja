package frc.robot.subsystems.Claw.ClawRoller;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Ports;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemConstants;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemConstants.simType;

/** Add your docs here. */
public final class ClawRollerConstants {

    public static final GenericMotionProfiledSubsystemConstants kSubSysConstants =
        new GenericMotionProfiledSubsystemConstants();

    static {
        kSubSysConstants.kName = "ClawRoller";

        kSubSysConstants.kLeaderMotor = Ports.CLAW_ROLLER;
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

        kSubSysConstants.kMotorConfig.CurrentLimits.SupplyCurrentLimit = 60;
        kSubSysConstants.kMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        kSubSysConstants.kMotorConfig.CurrentLimits.StatorCurrentLimit = 70;
        kSubSysConstants.kMotorConfig.CurrentLimits.StatorCurrentLimitEnable = false;

        /* REAL system profile constants */

        kSubSysConstants.kMotorConfig.Slot0.kP = 500;
        kSubSysConstants.kMotorConfig.Slot0.kI = 0;
        kSubSysConstants.kMotorConfig.Slot0.kD = 5;
        kSubSysConstants.kMotorConfig.Slot0.kG = 0;
        kSubSysConstants.kMotorConfig.Slot0.kS = 9;
        kSubSysConstants.kMotorConfig.Slot0.kV = 0;
        kSubSysConstants.kMotorConfig.Slot0.kA = 0;
        kSubSysConstants.kMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 100;
        kSubSysConstants.kMotorConfig.MotionMagic.MotionMagicAcceleration = 10;
        kSubSysConstants.kMotorConfig.MotionMagic.MotionMagicJerk = 0;
        kSubSysConstants.kMotorConfig.Slot1.kP = 0;
        kSubSysConstants.kMotorConfig.Slot1.kI = 0;
        kSubSysConstants.kMotorConfig.Slot1.kD = 0;
        kSubSysConstants.kMotorConfig.Slot1.kG = 0;
        kSubSysConstants.kMotorConfig.Slot1.kS = 0;
        kSubSysConstants.kMotorConfig.Slot1.kV = 0;
        kSubSysConstants.kMotorConfig.Slot1.kA = 0;

        /* SIM system profile constants */
        kSubSysConstants.kSimMotorConfig.Slot0.kP = 500;
        kSubSysConstants.kSimMotorConfig.Slot0.kI = 0;
        kSubSysConstants.kSimMotorConfig.Slot0.kD = 20;
        kSubSysConstants.kSimMotorConfig.Slot0.kG = 0;
        kSubSysConstants.kSimMotorConfig.Slot0.kS = 0;
        kSubSysConstants.kSimMotorConfig.Slot0.kV = 0.19;
        kSubSysConstants.kSimMotorConfig.Slot0.kA = 0;
        kSubSysConstants.kSimMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 500;
        kSubSysConstants.kSimMotorConfig.MotionMagic.MotionMagicAcceleration = 50;
        kSubSysConstants.kSimMotorConfig.MotionMagic.MotionMagicJerk = 0;

        // Simulation Type
        kSubSysConstants.SimType = simType.ROLLER;

        // Motor simulation
        kSubSysConstants.kMotorSimConfig.simMotorModelSupplier = () -> DCMotor.getKrakenX60Foc(1);
        kSubSysConstants.kMotorSimConfig.simReduction = 1;
    }
}
