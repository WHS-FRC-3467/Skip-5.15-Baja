package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.TunablePIDController;
import frc.robot.util.TuneableProfiledPID;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

/**
 * A command that drives the robot to a target {@link Pose2d} using PID control and motion
 * profiling.
 * <p>
 * This command generates linear and angular motion profiles to smoothly drive toward the target
 * pose. It supports tolerance checking, dynamic velocity/acceleration limits based on elevator
 * extension, and manual overrides or feedforwards for advanced control.
 *
 * <p>
 * Features:
 * <ul>
 * <li>Trapezoidal profiling for linear and angular motion</li>
 * <li>PID control for both translation and rotation</li>
 * <li>Support for dynamic tuning of PID and constraints via {@link LoggedTunableNumber}</li>
 * <li>Overrides and feedforwards for advanced driver or autonomous control</li>
 * </ul>
 */
public class DriveToPose extends Command {

    // === Subsystems and pose suppliers ===

    private final Drive drive; // Drive subsystem
    private final Supplier<Pose2d> target; // Target pose supplier
    private Supplier<Pose2d> robot; // Robot pose supplier (usually from drive)

    // === Tolerances and state tracking ===

    private double driveTolerance = 0.0;
    private Rotation2d thetaTolerance = Rotation2d.kZero;
    private boolean finishWithinTolerance = true;
    @Getter
    private boolean running = false;

    // === Controllers and profiles ===

    private final TunablePIDController driveController =
        new TunablePIDController("DriveToPose/DriveController", 3.0, 0.0, 0.1);
    private final TuneableProfiledPID thetaController =
        new TuneableProfiledPID("DriveToPose/ThetaController", 4.0, 0.0, 0.0, 0.0, 0.0);
    private TrapezoidProfile driveProfile;

    // === Feedforward and override support ===

    private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
    private DoubleSupplier omegaFF = () -> 0.0;
    private Supplier<Translation2d> linearOverride = () -> Translation2d.kZero;
    private DoubleSupplier omegaOverride = () -> 0.0;

    // === Setpoint tracking ===

    private Translation2d lastSetpointTranslation = Translation2d.kZero;
    private Translation2d lastSetpointVelocity = Translation2d.kZero;
    private Rotation2d lastGoalRotation = Rotation2d.kZero;
    private double lastTime = 0.0;
    private double driveErrorAbs = 0.0;
    private double thetaErrorAbs = 0.0;

    // === Tunable motion limits ===

    private final LoggedTunableNumber driveMaxVelocity =
        new LoggedTunableNumber("DriveToPose/DriveMaxVelocity", 3);
    private final LoggedTunableNumber driveMaxVelocityTop =
        new LoggedTunableNumber("DriveToPose/DriveMaxVelocityTop", 2);
    private final LoggedTunableNumber driveMaxAcceleration =
        new LoggedTunableNumber("DriveToPose/DriveMaxAcceleration", 4);
    private final LoggedTunableNumber driveMaxAccelerationTop =
        new LoggedTunableNumber("DriveToPose/DriveMaxAccelerationTop", 3);
    private final LoggedTunableNumber thetaMaxVelocity =
        new LoggedTunableNumber("DriveToPose/ThetaMaxVelocity", 9.27);
    private final LoggedTunableNumber thetaMaxVelocityTop =
        new LoggedTunableNumber("DriveToPose/ThetaMaxVelocityTop", 2.31);
    private final LoggedTunableNumber thetaMaxAcceleration =
        new LoggedTunableNumber("DriveToPose/ThetaMaxAcceleration", 92.7);
    private final LoggedTunableNumber thetaMaxAccelerationTop =
        new LoggedTunableNumber("DriveToPose/ThetaMaxAccelerationTop", 4);

    /**
     * Constructs a new DriveToPose command using the given drive subsystem and target pose
     * supplier.
     *
     * @param drive The drive subsystem.
     * @param target The pose to drive to.
     */
    public DriveToPose(Drive drive, Supplier<Pose2d> target)
    {
        this.drive = drive;
        this.robot = drive::getPose;
        this.target = target;

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drive);
    }

    /**
     * Constructs a new DriveToPose with an alternate robot pose supplier.
     *
     * @param drive The drive subsystem.
     * @param target The pose to drive to.
     * @param robot Robot's current pose supplier.
     */
    public DriveToPose(Drive drive, Supplier<Pose2d> target, Supplier<Pose2d> robot)
    {
        this(drive, target);
        this.robot = robot;
    }

    /**
     * Sets feedforward terms for linear and angular motion.
     */
    public DriveToPose withFeedforward(Supplier<Translation2d> linearFF, DoubleSupplier omegaFF)
    {
        this.linearFF = linearFF;
        this.omegaFF = omegaFF;
        return this;
    }

    /**
     * Sets driver override inputs for linear and angular control.
     */
    public DriveToPose withOverride(Supplier<Translation2d> linearOverride,
        DoubleSupplier omegaOverride)
    {
        this.linearOverride = linearOverride;
        this.omegaOverride = omegaOverride;
        return this;
    }

    /**
     * Sets tolerances for determining when the command is finished.
     */
    public DriveToPose withTolerance(double driveTolerance, Rotation2d thetaTolerance)
    {
        this.driveTolerance = driveTolerance;
        this.thetaTolerance = thetaTolerance;
        driveController.setTolerance(driveTolerance);
        thetaController.setTolerance(thetaTolerance.getRadians());
        return this;
    }

    /**
     * Configures whether this command should end automatically when within tolerance.
     */
    public DriveToPose finishWithinTolerance(boolean finish)
    {
        this.finishWithinTolerance = finish;
        return this;
    }

    /** Initializes the PID controllers and motion profile from current state. */
    @Override
    public void initialize()
    {
        // Get the current pose of the robot and the target pose to drive toward
        Pose2d currentPose = robot.get();
        Pose2d targetPose = target.get();

        // Get the robot's current field-relative velocity
        ChassisSpeeds fieldVelocity = drive.getChassisSpeeds();
        Translation2d linearFieldVelocity =
            new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);

        // Initialize the drive motion profile with current velocity and acceleration constraints
        driveProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()));

        // Configure the drive and theta (rotation) controllers with tolerance values
        driveController.setTolerance(driveTolerance);
        thetaController.setTolerance(thetaTolerance.getRadians());

        // Reset the controllers to begin a new motion
        driveController.reset();
        thetaController.reset(
            currentPose.getRotation().getRadians(),
            fieldVelocity.omegaRadiansPerSecond);

        // Store initial conditions for use in motion profiling
        lastSetpointTranslation = currentPose.getTranslation(); // Starting position
        lastSetpointVelocity = linearFieldVelocity; // Starting linear velocity
        lastGoalRotation = targetPose.getRotation(); // Initial target orientation
        lastTime = Timer.getTimestamp(); // Record the current timestamp
    }

    /** Executes one cycle of the PID update and motion profiling. */
    @Override
    public void execute()
    {
        // Update PID based on tunable numbers
        driveController.updatePID();
        thetaController.updatePID();

        running = true;

        // Dynamically adjust motion profile constraints based on elevator extension
        double extensionS = RobotState.getInstance().getElevatorExtensionPercent();
        driveProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            MathUtil.interpolate(driveMaxVelocity.get(), driveMaxVelocityTop.get(), extensionS),
            MathUtil.interpolate(driveMaxAcceleration.get(), driveMaxAccelerationTop.get(),
                extensionS)));
        thetaController.setConstraints(new TrapezoidProfile.Constraints(
            MathUtil.interpolate(thetaMaxVelocity.get(), thetaMaxVelocityTop.get(), extensionS),
            MathUtil.interpolate(thetaMaxAcceleration.get(), thetaMaxAccelerationTop.get(),
                extensionS)));

        // Retrieve current and target robot poses
        Pose2d currentPose = robot.get();
        Pose2d targetPose = target.get();

        // Calculate positional and angular error
        Pose2d poseError = currentPose.relativeTo(targetPose);
        driveErrorAbs = poseError.getTranslation().getNorm();
        thetaErrorAbs = Math.abs(poseError.getRotation().getRadians());

        // Calculate scaling factors for motion profile feedforward based on distance from target
        double linearFFScaler = MathUtil.clamp((driveErrorAbs - 0.01) / (0.5 - 0.01), 0.0, 1.0);

        // --- Compute linear velocity toward the target ---

        // Determine motion direction from last setpoint
        var direction = targetPose.getTranslation().minus(lastSetpointTranslation).toVector();

        // Estimate setpoint velocity in the direction of travel
        double setpointVelocity = direction.norm() <= 0.01
            ? lastSetpointVelocity.getNorm()
            : Math.abs(lastSetpointVelocity.toVector().dot(direction)) / direction.norm();
        setpointVelocity = Math.max(setpointVelocity, -0.5);

        // Calculate position and velocity setpoint using trapezoidal profile
        State driveSetpoint = driveProfile.calculate(
            Constants.loopPeriodSecs,
            new State(direction.norm(), -setpointVelocity), // Inverted: target is the profile's
                                                            // zero point
            new State(0.0, 0.0));

        // Apply PID + feedforward to determine desired linear velocity
        double driveVelocityScalar =
            driveController.calculate(driveErrorAbs, driveSetpoint.position)
                + driveSetpoint.velocity * linearFFScaler;

        // Calculate vector pointing from target to current position
        Rotation2d targetToCurrentAngle =
            currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle();
        Translation2d driveVelocity = new Translation2d(driveVelocityScalar, targetToCurrentAngle);

        // Update last setpoint info
        lastSetpointTranslation = new Pose2d(targetPose.getTranslation(), targetToCurrentAngle)
            .transformBy(new Transform2d(driveSetpoint.position, 0.0, Rotation2d.kZero))
            .getTranslation();
        lastSetpointVelocity = new Translation2d(driveSetpoint.velocity, targetToCurrentAngle);

        // --- Compute rotational velocity (theta) ---

        // Estimate target angular velocity based on change in goal rotation
        double thetaSetpointVelocity =
            Math.abs(targetPose.getRotation().minus(lastGoalRotation).getDegrees()) < 10.0
                ? (targetPose.getRotation().minus(lastGoalRotation)).getRadians()
                    / (Timer.getTimestamp() - lastTime)
                : thetaController.getSetpoint().velocity;

        // Apply PID + feedforward to determine desired angular velocity
        double thetaVelocity = thetaController.calculate(
            currentPose.getRotation().getRadians(),
            new State(targetPose.getRotation().getRadians(), thetaSetpointVelocity))
            + thetaController.getSetpoint().velocity;

        // Zero out small errors for stability
        if (thetaErrorAbs < thetaController.getPositionTolerance()) {
            thetaVelocity = 0.0;
        }

        // Update last goal and timestamp
        lastGoalRotation = targetPose.getRotation();
        lastTime = Timer.getTimestamp();

        // --- Manual input override blending (e.g. joystick assist) ---

        if (!linearOverride.get().equals(Translation2d.kZero)) {
            final double linearS = MathUtil.clamp(linearOverride.get().getNorm() * 3.0, 0.0, 1.0);

            // Blend PID output with manual input
            driveVelocity = driveVelocity.interpolate(
                linearOverride.get().times(driveMaxVelocity.get()), linearS);

            // Reset profile tracking if override is significant
            if (linearS >= 0.2) {
                ChassisSpeeds fieldVelocity = drive.getChassisSpeeds();
                Translation2d linearFieldVelocity = new Translation2d(
                    fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
                lastSetpointTranslation = currentPose.getTranslation();
                lastSetpointVelocity = linearFieldVelocity;
            }
        }

        if (omegaOverride.getAsDouble() != 0.0) {
            final double thetaS =
                MathUtil.clamp(Math.abs(omegaOverride.getAsDouble()) * 3.0, 0.0, 1.0);

            // Blend PID output with manual angular input
            thetaVelocity = MathUtil.interpolate(
                thetaVelocity, omegaOverride.getAsDouble() * thetaMaxVelocity.get(), thetaS);

            // Reset theta profile if input is significant
            if (thetaS >= 0.1) {
                ChassisSpeeds fieldVelocity = drive.getChassisSpeeds();
                thetaController.reset(
                    currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
            }
        }

        // --- Apply feedforward additions ---

        driveVelocity = driveVelocity.plus(linearFF.get().times(driveMaxVelocity.get()));
        thetaVelocity += omegaFF.getAsDouble() * thetaMaxVelocity.get();

        // Command the chassis with the computed velocities in field-relative frame
        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));

        // Logging for diagnostics and tuning
        Logger.recordOutput("DriveToPose/DistanceMeasured", driveErrorAbs);
        Logger.recordOutput("DriveToPose/DistanceSetpoint", driveSetpoint.position);
        Logger.recordOutput("DriveToPose/VelocitySetpoint", driveSetpoint.velocity);
        Logger.recordOutput("DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
        Logger.recordOutput("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
        Logger.recordOutput("DriveToPose/Setpoint", new Pose2d[] {
                new Pose2d(lastSetpointTranslation,
                    Rotation2d.fromRadians(thetaController.getSetpoint().position))
        });
        Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {targetPose});
        Logger.recordOutput("DriveToPose/IOutput",
            driveController.getAccumulatedError() * driveController.getI());
    }


    /**
     * Returns true if the robot is within the configured tolerances.
     */
    public boolean withinTolerance()
    {
        return running && driveErrorAbs <= driveTolerance
            && thetaErrorAbs <= thetaTolerance.getRadians();
    }

    /**
     * Returns true if the robot is within the specified tolerances.
     */
    public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance)
    {
        return running && driveErrorAbs <= driveTolerance
            && thetaErrorAbs <= thetaTolerance.getRadians();
    }

    /**
     * Ends the command by stopping the drive and resetting logs.
     */
    @Override
    public void end(boolean interrupted)
    {
        drive.stop();
        running = false;
        Logger.recordOutput("DriveToPose/Setpoint", new Pose2d[] {});
        Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {});
    }

    /**
     * Returns true if the robot is within tolerance and the command is configured to end.
     */
    @Override
    public boolean isFinished()
    {
        return finishWithinTolerance && withinTolerance();
    }
}
