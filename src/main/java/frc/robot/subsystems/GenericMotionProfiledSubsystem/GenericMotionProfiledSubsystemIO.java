package frc.robot.subsystems.GenericMotionProfiledSubsystem;

import org.littletonrobotics.junction.AutoLog;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem.ProfileType;

public interface GenericMotionProfiledSubsystemIO {
    @AutoLog
    abstract class GenericMotionProfiledIOInputs {

        // Flags to indicate which components are active and connected
        public boolean leaderMotorConnected = false;
        public boolean followerMotorConnected = false;
        public boolean CANcoderConnected = false;

        public double positionRot = 0.0;
        public double velocityRps = 0.0;
        public double[] appliedVoltage = new double[] {0.0, 0.0};
        public double[] supplyCurrentAmps = new double[] {0.0, 0.0};
        public double[] torqueCurrentAmps = new double[] {0.0, 0.0};
        public double[] tempCelsius = new double[] {0.0, 0.0};

        public double errorRotations = 0.0;
        public double activeTrajectoryPosition = 0.0;
        public double activeTrajectoryVelocity = 0.0;

        public double absoluteEncoderPositionRot = 0.0;
        public double relativeEncoderPositionRot = 0.0;
    }

    default void updateInputs(GenericMotionProfiledIOInputs inputs)
    {}

    /** Run Open Loop at the specified voltage */
    public default void runVoltage(double volts)
    {}

    /** Run Open Loop at the specified current */
    public default void runCurrent(double amps, double maxDutyCycle)
    {}

    /** Run Closed Loop to position in rotations */
    public default void runToPosition(double position, int slot)
    {}

    /** Run Closed Loop to velocity in rotations/second */
    public default void runToVelocity(double velocity, int slot)
    {}

    /** Run Motion Magic to the specified setpoint */
    public default void runMotionMagicPosition(double setpoint, int slot)
    {}

    /** Run Motion Magic to the specified velocity */
    public default void runMotionMagicVelocity(double velocity, int slot)
    {}

    /* Stop in Coast mode */
    public default void stopCoast()
    {}

    /* Stop in Brake mode */
    public default void stopBrake()
    {}

    /* Stop in Open Loop */
    public default void stop()
    {}

    /* Configure PID constants */
    public default void configurePID(double kP, double kI, double kD, boolean check)
    {}

    /* Configure Closed Loop constants */
    public default void configureGSVA(double kG, double kS, double kV, double kA, boolean check)
    {}

    /* Configure Motion constants */
    public default void configureMotion(double kCruise, double kAccel, double kJerk, boolean check)
    {}

    /* Get the closed loop operational setpoint (in rotations) */
    public default double getSetpoint()
    {
        return 0;
    }

    /* Get current mechanism position (in rotations) */
    public default double getPosition()
    {
        return 0;
    }

    /* Get latest trajectory position (in rotations) */
    public default double getCurrTrajPos()
    {
        return 0;
    }

    /* Get current mechanism velocity (in rotations per second) */
    public default double getVelocity()
    {
        return 0;
    }

    /* Get current lead motor supply current) */
    public default double getSupplyCurrent()
    {
        return 0;
    }

    /* Has the closed loop completed (within tolerance)? */
    public default boolean atPosition(ProfileType profileType, double tolerance)
    {
        return false;
    }

    /* Encoder Homing and Reset */
    public default void zeroSensors()
    {}

    public default boolean hasBeenZeroed()
    {
        return false;
    }
}
