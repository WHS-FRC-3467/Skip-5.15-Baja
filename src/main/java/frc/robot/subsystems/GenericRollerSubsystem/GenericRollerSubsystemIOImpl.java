package frc.robot.subsystems.GenericRollerSubsystem;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.drivers.Phoenix6Util;
import frc.robot.util.sim.PhysicsSim;
import frc.robot.util.sim.mechanisms.GenericRollerSimMechanism;
import java.util.ArrayList;
import java.util.List;

/** Generic roller IO implementation for a roller or series of rollers using a Kraken. */
public abstract class GenericRollerSubsystemIOImpl implements GenericRollerSubsystemIO {

    private boolean mInitialized = false;
    private boolean mIsSim = false;

    private PhysicsSim mSim = null;
    private GenericRollerSimMechanism mMechanism = null;

    // Local motor object(s)
    private List<TalonFX> mMotors = new ArrayList<TalonFX>();
    private int mNumMotors = 0;

    // Local motor config
    private TalonFXConfiguration mConfig;

    // All the Talon StatusSignals of interest
    private final List<StatusSignal<Angle>> mPositionRot = new ArrayList<StatusSignal<Angle>>();
    private final List<StatusSignal<AngularVelocity>> mVelocityRps =
        new ArrayList<StatusSignal<AngularVelocity>>();
    private final List<StatusSignal<Voltage>> mAppliedVoltage =
        new ArrayList<StatusSignal<Voltage>>();
    private final List<StatusSignal<Current>> mSupplyCurrent =
        new ArrayList<StatusSignal<Current>>();
    private final List<StatusSignal<Current>> mTorqueCurrent =
        new ArrayList<StatusSignal<Current>>();
    private final List<StatusSignal<Temperature>> mTempCelsius =
        new ArrayList<StatusSignal<Temperature>>();

    // Single shot for voltage mode, robot loop will call continuously
    private final VoltageOut mVoltageOut =
        new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);

    /*
     * Constructor
     */
    public GenericRollerSubsystemIOImpl(GenericRollerSubsystemConstants constants, boolean isSim)
    {

        GenericRollerSubsystemConstants mConst = constants;
        this.mIsSim = isSim;

        // Number of motors is based on how many IDs are specified in the provided
        // Constants
        mNumMotors = mConst.kMotorIDs.size();

        if (mIsSim) {
            // Create the Physics Sim and the Mechanism display
            mSim = PhysicsSim.getInstance();
            mMechanism = new GenericRollerSimMechanism(mConst.kName, mNumMotors);
        }

        // Instantiate each roller
        for (int i = 0; i < mNumMotors; i++) {

            // Instantiate the TalonFX object for this roller
            TalonFX mMotor =
                new TalonFX(mConst.kMotorIDs.get(i).getDeviceNumber(),
                    mConst.kMotorIDs.get(i).getBus());
            mMotors.add(mMotor);

            // Get the motor configuration group and configure the motor
            mConfig = mConst.kMotorConfig;
            Phoenix6Util.applyAndCheckConfiguration(mMotor, mConfig);

            // Assign StatusSignals to our local variables
            mPositionRot.add(mMotor.getPosition());
            mVelocityRps.add(mMotor.getVelocity());
            mAppliedVoltage.add(mMotor.getMotorVoltage());
            mSupplyCurrent.add(mMotor.getSupplyCurrent());
            mTorqueCurrent.add(mMotor.getTorqueCurrent());
            mTempCelsius.add(mMotor.getDeviceTemp());

            // Set update frequencies for some basic output signals
            Phoenix6Util
                .checkErrorAndRetry(() -> mMotor.getBridgeOutput().setUpdateFrequency(200, 0.05));
            Phoenix6Util
                .checkErrorAndRetry(() -> mMotor.getFault_Hardware().setUpdateFrequency(4, 0.05));

            // Set update frequencies for the StatusSignals of interest
            final int index = i;
            Phoenix6Util.checkErrorAndRetry(
                () -> BaseStatusSignal.setUpdateFrequencyForAll(
                    100,
                    mPositionRot.get(index),
                    mVelocityRps.get(index),
                    mAppliedVoltage.get(index),
                    mSupplyCurrent.get(index),
                    mTorqueCurrent.get(index),
                    mTempCelsius.get(index)));

            mMotor.optimizeBusUtilization(0, 1.0);

            if (mIsSim) {
                // Add the motor object to the Physics Sim
                mSim.addTalonFX(
                    mMotor,
                    new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(
                            constants.simMotorModelSupplier.get(),
                            constants.simMOI,
                            constants.simReduction),
                        constants.simMotorModelSupplier.get()));
            }
        }

        // Send a neutral command to halt all motors.
        stop();
    }

    @Override
    public void updateInputs(GenericRollerIOInputs inputs)
    {

        /*
         * Because this code supports a configurable number of motors, we must wait until the first
         * time this method is called to instantiate arrays of the proper size
         */
        if (!mInitialized) {
            inputs.connected = new boolean[mNumMotors];
            inputs.positionRot = new double[mNumMotors];
            inputs.velocityRps = new double[mNumMotors];
            inputs.appliedVoltage = new double[mNumMotors];
            inputs.supplyCurrentAmps = new double[mNumMotors];
            inputs.torqueCurrentAmps = new double[mNumMotors];
            inputs.tempCelsius = new double[mNumMotors];

            mInitialized = true;
        }

        if (mIsSim) {
            // Run the Sim
            mSim.run();
        }

        for (int i = 0; i < mNumMotors; i++) {

            if (DriverStation.isDisabled()) {
                runVolts(i, 0.0);
            }

            // Check & Refresh all signals of interest
            inputs.connected[i] =
                BaseStatusSignal.refreshAll(
                    mPositionRot.get(i),
                    mVelocityRps.get(i),
                    mAppliedVoltage.get(i),
                    mSupplyCurrent.get(i),
                    mTorqueCurrent.get(i),
                    mTempCelsius.get(i))
                    .isOK();

            // Get velocity, voltage, currents, and temperature for the motor
            inputs.positionRot[i] = mPositionRot.get(i).getValueAsDouble();
            inputs.velocityRps[i] = mVelocityRps.get(i).getValueAsDouble();
            inputs.appliedVoltage[i] = mAppliedVoltage.get(i).getValueAsDouble();
            inputs.supplyCurrentAmps[i] = mSupplyCurrent.get(i).getValueAsDouble();
            inputs.torqueCurrentAmps[i] = mTorqueCurrent.get(i).getValueAsDouble();
            inputs.tempCelsius[i] = mTempCelsius.get(i).getValueAsDouble();

            if (mIsSim) {
                // Update the sim mechanism
                mMechanism.update(i, inputs.positionRot[i]);
            }
        }
    }

    /** Run roller at specified voltage */
    @Override
    public void runVolts(int index, double volts)
    {
        mMotors.get(index).setControl(mVoltageOut.withOutput(volts));
    }

    /** Stop in Open Loop */
    @Override
    public void stop()
    {
        for (int i = 0; i < mNumMotors; i++) {
            runVolts(i, 0.0);
            mMotors.get(i).stopMotor();
        }
    }
}
