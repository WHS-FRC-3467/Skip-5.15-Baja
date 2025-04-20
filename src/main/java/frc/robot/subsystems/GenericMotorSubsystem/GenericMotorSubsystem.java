package frc.robot.subsystems.GenericMotorSubsystem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public abstract class GenericMotorSubsystem<G extends GenericMotorSubsystem.TargetState>
    extends SubsystemBase {

    // Tunable numbers
    private LoggedTunableNumber kP, kI, kD, kG, kS, kV, kA, kCruiseVelocity, kAcceleration, kJerk;

    public sealed interface ProfileType {
        record POSITION(DoubleSupplier position, int slot) implements ProfileType {
        }
        record VELOCITY(DoubleSupplier velocity, int slot) implements ProfileType {
        }
        record MM_POSITION(DoubleSupplier position, int slot) implements ProfileType {
        }
        record MM_VELOCITY(DoubleSupplier velocity, int slot) implements ProfileType {
        }
        record OPEN_VOLTAGE(DoubleSupplier voltage) implements ProfileType {
        }
        record OPEN_CURRENT(DoubleSupplier current, DoubleSupplier maxDutyCycle)
            implements ProfileType {
        }
        record DISABLED_COAST() implements ProfileType {
        }
        record DISABLED_BRAKE() implements ProfileType {
        }
        record CHARACTERIZATION() implements ProfileType {
        }
    }

    public interface TargetState {
        public ProfileType getProfileType();
    }

    public abstract G getState();

    private final String name;
    private final GenericMotorSubsystemConstants constants;
    protected final GenericMotorSubsystemIO io;
    private ProfileType proType;

    protected final GenericMotorIOInputsAutoLogged inputs =
        new GenericMotorIOInputsAutoLogged();
    private final Alert leaderMotorDisconnected;
    private final Alert followerMotorDisconnected;
    private final Alert CANcoderDisconnected;

    public GenericMotorSubsystem(
        ProfileType defaultProfileType,
        GenericMotorSubsystemConstants constants,
        GenericMotorSubsystemIO io,
        boolean isSim)
    {

        this.proType = defaultProfileType;
        this.constants = constants;
        this.io = io;
        this.name = constants.kName;

        this.leaderMotorDisconnected =
            new Alert(name + " Leader motor disconnected!", Alert.AlertType.kWarning);
        this.followerMotorDisconnected =
            new Alert(name + " Follower motor disconnected!", Alert.AlertType.kWarning);
        this.CANcoderDisconnected =
            new Alert(name + " CANcoder disconnected!", Alert.AlertType.kWarning);

        // Make sure we use the correct profiling configs
        TalonFXConfiguration fxConfig =
            isSim ? constants.kSimMotorConfig : constants.kMotorConfig;

        // Tunable numbers for PID and motion gain constants
        kP = new LoggedTunableNumber(name + "/Gains/kP", fxConfig.Slot0.kP);
        kI = new LoggedTunableNumber(name + "/Gains/kI", fxConfig.Slot0.kI);
        kD = new LoggedTunableNumber(name + "/Gains/kD", fxConfig.Slot0.kD);

        kG = new LoggedTunableNumber(name + "/Gains/kG", fxConfig.Slot0.kG);
        kS = new LoggedTunableNumber(name + "/Gains/kS", fxConfig.Slot0.kS);
        kV = new LoggedTunableNumber(name + "/Gains/kV", fxConfig.Slot0.kV);
        kA = new LoggedTunableNumber(name + "/Gains/kA", fxConfig.Slot0.kA);

        kCruiseVelocity =
            new LoggedTunableNumber(
                name + "/CruiseVelocity", fxConfig.MotionMagic.MotionMagicCruiseVelocity);
        kAcceleration =
            new LoggedTunableNumber(
                name + "/Acceleration", fxConfig.MotionMagic.MotionMagicAcceleration);
        kJerk = new LoggedTunableNumber(name + "/Jerk", fxConfig.MotionMagic.MotionMagicJerk);

        io.configurePID(kP.get(), kI.get(), kD.get(), true);
        io.configureGSVA(kG.get(), kS.get(), kV.get(), kA.get(), true);
        io.configureMotion(kCruiseVelocity.get(), kAcceleration.get(), kJerk.get(), true);
    }

    public void periodic()
    {
        // If Profile Type has changed, reset the encoder(s)
        ProfileType newProfType = getState().getProfileType();
        if (proType.getClass() != newProfType.getClass()) {
            // io.zeroSensors();
        }
        if (proType != newProfType) {
            proType = newProfType;
        }

        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);

        // Check for disconnections
        leaderMotorDisconnected.set(!inputs.leaderMotorConnected);
        followerMotorDisconnected.set(
            constants.kFollowMotor != null && !inputs.followerMotorConnected);
        CANcoderDisconnected.set(constants.kCANcoder != null && !inputs.CANcoderConnected);

        // If changed, update controller constants from Tuneable Numbers
        if (kP.hasChanged(hashCode()) || kI.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
            io.configurePID(kP.get(), kI.get(), kD.get(), true);
        }

        if (kG.hasChanged(hashCode())
            || kS.hasChanged(hashCode())
            || kV.hasChanged(hashCode())
            || kA.hasChanged(hashCode())) {
            io.configureGSVA(kG.get(), kS.get(), kV.get(), kA.get(), true);
        }

        if (kCruiseVelocity.hasChanged(hashCode())
            || kAcceleration.hasChanged(hashCode())
            || kJerk.hasChanged(hashCode())) {
            io.configureMotion(kCruiseVelocity.get(), kAcceleration.get(), kJerk.get(), true);
        }

        // Run system based on Profile Type
        if (proType instanceof ProfileType.POSITION profile) {
            /* Run Closed Loop to position in rotations */
            io.runToPosition(profile.position.getAsDouble(), profile.slot);
        } else if (proType instanceof ProfileType.VELOCITY profile) {
            /* Run Closed Loop to velocity in rotations/second */
            io.runToVelocity(profile.velocity.getAsDouble(), profile.slot);
        } else if (proType instanceof ProfileType.MM_POSITION profile) {
            /* Run Motion Magic to the specified position setpoint (in rotations) */
            io.runMotionMagicPosition(profile.position.getAsDouble(), profile.slot);
        } else if (proType instanceof ProfileType.MM_VELOCITY profile) {
            /* Run Motion Magic to the specified velocity setpoint (in rotations/second) */
            io.runMotionMagicVelocity(profile.velocity.getAsDouble(), profile.slot);
        } else if (proType instanceof ProfileType.OPEN_VOLTAGE profile) {
            /* Run Open Loop using specified voltage in volts */
            io.runVoltage(profile.voltage.getAsDouble());
        } else if (proType instanceof ProfileType.OPEN_CURRENT profile) {
            /* Run Open Loop using specified current in amps */
            io.runCurrent(profile.current.getAsDouble(), profile.maxDutyCycle.getAsDouble());
        } else if (proType instanceof ProfileType.DISABLED_COAST) {
            /* Stop all output and put motor in Coast mode */
            io.stopCoast();
        } else if (proType instanceof ProfileType.DISABLED_BRAKE) {
            /* Stop all output and put motor in Brake mode */
            io.stopBrake();
        } else if (proType instanceof ProfileType.CHARACTERIZATION) {
            /*
             * Run Open Loop for characterization in the child subsystem class's characterization
             * command. Do nothing here.
             */
        }

        displayInfo();
    }

    private void displayInfo()
    {

        Logger.recordOutput(name + "/Goal State", getState().toString());
        Logger.recordOutput(name + "/Profile Type", getState().getProfileType().toString());
        // As the Arm is the only non-drive subsystem with an external CANcoder, report if Arm is in
        // fallback
        if (constants.kCANcoder != null) {
            SmartDashboard.putBoolean("Arm Fallback Active",
                ((constants.kCANcoder != null) && (!inputs.CANcoderConnected)));
            Logger.recordOutput(name + "/Fallback Active", !inputs.CANcoderConnected);
        }

        if (Constants.tuningMode) {
            Logger.recordOutput(name + "/Setpoint", io.getSetpoint());
            Logger.recordOutput(name + "/Position(Rotations)", io.getPosition());
            Logger.recordOutput(name + "/Position(Degrees)",
                (Units.rotationsToDegrees(io.getPosition())));
            Logger.recordOutput(name + "/Velocity", io.getVelocity());
            Logger.recordOutput(name + "/CurrTrajPos", io.getCurrTrajPos());
            Logger.recordOutput(name + "/AtPosition?", io.atPosition(proType, 0.0));
            Logger.recordOutput(name + "/Appl Volt", inputs.appliedVoltage[0]);
            Logger.recordOutput(name + "/Supply Current", inputs.supplyCurrentAmps[0]);
        }
    }
}
