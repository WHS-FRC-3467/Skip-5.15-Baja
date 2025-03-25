package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemConstants;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemConstants.simType;

/** Add your docs here. */
public final class ArmConstants {

    public static final GenericMotionProfiledSubsystemConstants kSubSysConstants =
        new GenericMotionProfiledSubsystemConstants();

    public static final double kHomingCurrent = 2.0;

    static {
        kSubSysConstants.kName = "Arm";

        // This is the minimum tolerance that will be used by the atPosition() method.
        // It will be used even if you pass a smaller value into atPosition().
        // If you want to specify a larger value on an individual call basis, then you
        // should pass that value into atPosition()
        kSubSysConstants.kminTolerance = 0.01;

        kSubSysConstants.kLeaderMotor = Ports.ARM_MAIN;
        // kSubSysConstants.kFollowMotor = Ports.ARM_FOLLOWER;
        // kSubSysConstants.kFollowerOpposesMain = true;

        // Using TalonFX internal encoder

        // kSubSysConstants.kCANcoder = null;
        // kSubSysConstants.kMotorConfig.Feedback.FeedbackSensorSource =
        // FeedbackSensorSourceValue.RotorSensor;
        // kSubSysConstants.kMotorConfig.Feedback.SensorToMechanismRatio = 54.4;
        // kSubSysConstants.kMotorConfig.Feedback.RotorToSensorRatio = 1.0;

        // Using a remote CANcoder

        kSubSysConstants.kCANcoder = Ports.ARM_CANCODER;
        kSubSysConstants.kMotorConfig.Feedback.FeedbackRemoteSensorID =
            Ports.ARM_CANCODER.getDeviceNumber();
        kSubSysConstants.kMotorConfig.Feedback.FeedbackSensorSource =
            FeedbackSensorSourceValue.RemoteCANcoder;
        kSubSysConstants.kMotorConfig.Feedback.SensorToMechanismRatio = 1;
        kSubSysConstants.kMotorConfig.Feedback.RotorToSensorRatio = (9 / 1) * (48 / 22) * (70 / 22);
        // Different encoder offsets for each robot
        double kGortCANcoderOffset = 0.826416015625;
        double kBajaCANcoderOffset = -0.575439453125;
        kSubSysConstants.kEncoderConfig.MagnetSensor.MagnetOffset =
            (Constants.getRobot() == RobotType.GORT) ? kGortCANcoderOffset : kBajaCANcoderOffset;
        kSubSysConstants.kEncoderConfig.MagnetSensor.SensorDirection =
            SensorDirectionValue.Clockwise_Positive;
        kSubSysConstants.kEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;


        kSubSysConstants.kMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kSubSysConstants.kMotorConfig.MotorOutput.Inverted =
            InvertedValue.CounterClockwise_Positive;

        kSubSysConstants.kMotorConfig.Voltage.PeakForwardVoltage = 12.0;
        kSubSysConstants.kMotorConfig.Voltage.PeakReverseVoltage = -12.0;

        kSubSysConstants.kMotorConfig.CurrentLimits.SupplyCurrentLimit = 20;
        kSubSysConstants.kMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        kSubSysConstants.kMotorConfig.CurrentLimits.StatorCurrentLimit = 70;
        kSubSysConstants.kMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        kSubSysConstants.kMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.405;
        kSubSysConstants.kMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        kSubSysConstants.kMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        kSubSysConstants.kMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        /* REAL system profile constants */
        kSubSysConstants.kMotorConfig.Slot0.kP = 800;
        kSubSysConstants.kMotorConfig.Slot0.kI = 0;
        kSubSysConstants.kMotorConfig.Slot0.kD = 85;
        kSubSysConstants.kMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        kSubSysConstants.kMotorConfig.Slot0.kG = 12;
        kSubSysConstants.kMotorConfig.Slot0.kS = 4;
        kSubSysConstants.kMotorConfig.Slot0.kV = 0;
        kSubSysConstants.kMotorConfig.Slot0.kA = 0;
        kSubSysConstants.kMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 150;
        kSubSysConstants.kMotorConfig.MotionMagic.MotionMagicAcceleration = 80;
        kSubSysConstants.kMotorConfig.MotionMagic.MotionMagicJerk = 0;

        /* SIM system profile constants */
        kSubSysConstants.kSimMotorConfig.Slot0.kP = 1400;
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

        if (Robot.isSimulation()) {
            kSubSysConstants.kEncoderConfig.MagnetSensor.MagnetOffset = 0.0;
            kSubSysConstants.kMotorConfig.MotorOutput.Inverted =
                InvertedValue.Clockwise_Positive;

        }

        // Simulation Type
        kSubSysConstants.SimType = simType.ARM;

        // Motor simulation
        kSubSysConstants.kMotorSimConfig.simMotorModelSupplier = () -> DCMotor.getKrakenX60Foc(2);

        // Arm Simulation
        kSubSysConstants.kArmSimConfig.kIsComboSim = true;
        kSubSysConstants.kArmSimConfig.kArmMass = Units.lbsToKilograms(11); // Kilograms
        kSubSysConstants.kArmSimConfig.kArmLength = Units.inchesToMeters(14);
        kSubSysConstants.kArmSimConfig.kDefaultArmSetpointDegrees =
            Units.rotationsToDegrees(-0.405);
        kSubSysConstants.kArmSimConfig.kMinAngleDegrees = Units.rotationsToDegrees(-.405);
        kSubSysConstants.kArmSimConfig.kMaxAngleDegrees = 0;
        kSubSysConstants.kArmSimConfig.kArmReduction = (9 / 1) * (48 / 22) * (70 / 22); // RotorToSensorRatio
                                                                                        // *
                                                                                        // SensorToMechanismRatio
        kSubSysConstants.kArmSimConfig.kSensorReduction = 1; // SensorToMechanismRatio
    }
}
