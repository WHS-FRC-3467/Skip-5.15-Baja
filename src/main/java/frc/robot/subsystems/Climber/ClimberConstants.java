package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
// import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Ports;
import frc.robot.subsystems.GenericMotorSubsystem.GenericMotorSubsystemConstants;
import frc.robot.subsystems.GenericMotorSubsystem.GenericMotorSubsystemConstants.simType;

/** Add your docs here. */
public final class ClimberConstants {

    public static final GenericMotorSubsystemConstants kSubSysConstants =
        new GenericMotorSubsystemConstants();

    public static final double kSupplyCurrentLimit = 75.0;

    static {
        kSubSysConstants.kName = "Climber";

        kSubSysConstants.kLeaderMotor = Ports.CLIMBER;

        // Using TalonFX internal encoder
        kSubSysConstants.kCANcoder = null;
        kSubSysConstants.kMotorConfig.Feedback.FeedbackSensorSource =
            FeedbackSensorSourceValue.RotorSensor;
        kSubSysConstants.kMotorConfig.Feedback.SensorToMechanismRatio = 135.0;
        kSubSysConstants.kMotorConfig.Feedback.RotorToSensorRatio = 1.0;

        kSubSysConstants.kMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kSubSysConstants.kMotorConfig.MotorOutput.Inverted =
            InvertedValue.Clockwise_Positive;
        kSubSysConstants.kMotorConfig.Voltage.PeakForwardVoltage = 12.0;
        kSubSysConstants.kMotorConfig.Voltage.PeakReverseVoltage = -12.0;

        kSubSysConstants.kMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        kSubSysConstants.kMotorConfig.CurrentLimits.StatorCurrentLimitEnable = false;

        /* REAL system profile constants */
        kSubSysConstants.kMotorConfig.Slot0.kP = 40;
        kSubSysConstants.kMotorConfig.Slot1.kP = 1200;
        kSubSysConstants.kMotorConfig.Slot0.kI = 0;
        kSubSysConstants.kMotorConfig.Slot0.kD = 10;
        kSubSysConstants.kMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        kSubSysConstants.kMotorConfig.Slot0.kG = 0;
        kSubSysConstants.kMotorConfig.Slot0.kS = 0;
        kSubSysConstants.kMotorConfig.Slot0.kV = 0;
        kSubSysConstants.kMotorConfig.Slot0.kA = 0;
        kSubSysConstants.kMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 0.4;
        kSubSysConstants.kMotorConfig.MotionMagic.MotionMagicAcceleration = 1;
        kSubSysConstants.kMotorConfig.MotionMagic.MotionMagicJerk = 0;

        /* SIM system profile constants */
        kSubSysConstants.kSimMotorConfig.Slot0.kP = 700;
        kSubSysConstants.kSimMotorConfig.Slot0.kI = 0;
        kSubSysConstants.kSimMotorConfig.Slot0.kD = 100;
        kSubSysConstants.kSimMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        kSubSysConstants.kSimMotorConfig.Slot0.kG = 13;
        kSubSysConstants.kSimMotorConfig.Slot0.kS = 0;
        kSubSysConstants.kSimMotorConfig.Slot0.kV = 0.19;
        kSubSysConstants.kSimMotorConfig.Slot0.kA = 0;
        kSubSysConstants.kSimMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 500;
        kSubSysConstants.kSimMotorConfig.MotionMagic.MotionMagicAcceleration = 50;
        kSubSysConstants.kSimMotorConfig.MotionMagic.MotionMagicJerk = 0;


        // Simulation Type
        kSubSysConstants.SimType = simType.ARM;

        // Motor simulation
        kSubSysConstants.kMotorSimConfig.simMotorModelSupplier = () -> DCMotor.getKrakenX60Foc(1);

        // Climber Simulation
        kSubSysConstants.kArmSimConfig.kArmMass = Units.lbsToKilograms(4); // Kilograms
        kSubSysConstants.kArmSimConfig.kArmLength = Units.inchesToMeters(14);
        kSubSysConstants.kArmSimConfig.kDefaultArmSetpointDegrees = 90.0;
        kSubSysConstants.kArmSimConfig.kMinAngleDegrees = 0;
        kSubSysConstants.kArmSimConfig.kMaxAngleDegrees = 360;
        kSubSysConstants.kArmSimConfig.kArmReduction =
            1; // RotorToSensorRatio * SensorToMechanismRatio
        kSubSysConstants.kArmSimConfig.kSensorReduction = 1; // SensorToMechanismRatio
    }
}
