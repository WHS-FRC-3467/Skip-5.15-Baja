package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.TunablePIDController;
import frc.robot.util.TuneableProfiledPID;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {

    private final Drive drive;
    private final Supplier<Pose2d> target;

    private double driveTolerance = 0.0;
    private Rotation2d thetaTolerance = Rotation2d.kZero;

    private boolean finishWithinTolerance = true;

    private TrapezoidProfile driveProfile;
    private final PIDController driveController =
        new TunablePIDController("DriveToPose/DriveController", 1.0, 0.0, 0.0);
    private final ProfiledPIDController thetaController =
        new TuneableProfiledPID(
            "DriveToPose/ThetaController",
            4.0, 0.0, 0.0, 0.0, 0.0);

    private Translation2d lastSetpointTranslation = Translation2d.kZero;
    private Translation2d lastSetpointVelocity = Translation2d.kZero;
    private Rotation2d lastGoalRotation = Rotation2d.kZero;
    private double lastTime = 0.0;
    private double driveErrorAbs = 0.0;
    private double thetaErrorAbs = 0.0;
    @Getter
    private boolean running = false;
    private Supplier<Pose2d> robot;

    private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
    private DoubleSupplier omegaFF = () -> 0.0;

    private LoggedTunableNumber driveMaxVelocity =
        new LoggedTunableNumber("DriveToPose/DriveMaxVelocity", 3.79);
    private LoggedTunableNumber driveMaxVelocityTop =
        new LoggedTunableNumber("DriveToPose/DriveMaxVelocityTop", 1);
    private LoggedTunableNumber driveMaxAcceleration =
        new LoggedTunableNumber("DriveToPose/DriveMaxAcceleration", 4);
    private LoggedTunableNumber driveMaxAccelerationTop =
        new LoggedTunableNumber("DriveToPose/DriveMaxAccelerationTop", 1);
    private LoggedTunableNumber thetaMaxVelocity =
        new LoggedTunableNumber("DriveToPose/ThetaMaxVelocity", 3.79);
    private LoggedTunableNumber thetaMaxVelocityTop =
        new LoggedTunableNumber("DriveToPose/ThetaMaxVelocityTop", 1);
    private LoggedTunableNumber thetaMaxAcceleration =
        new LoggedTunableNumber("DriveToPose/ThetaMaxAcceleration", 4);
    private LoggedTunableNumber thetaMaxAccelerationTop =
        new LoggedTunableNumber("DriveToPose/ThetaMaxAccelerationTop", 1);

    public DriveToPose(Drive drive, Supplier<Pose2d> target)
    {
        this.drive = drive;
        this.robot = drive::getPose;
        this.target = target;

        // Enable continuous input for theta controller
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);
    }

    public DriveToPose(Drive drive, Supplier<Pose2d> target, Supplier<Pose2d> robot)
    {
        this(drive, target);
        this.robot = robot;
    }

    public DriveToPose(Drive drive, Supplier<Pose2d> target, Supplier<Pose2d> robot,
        Supplier<Translation2d> linearFF, DoubleSupplier omegaFF)
    {
        this(drive, target, robot);
        this.linearFF = linearFF;
        this.omegaFF = omegaFF;
    }

    public DriveToPose withTolerance(double driveTolerance, Rotation2d thetaTolerance)
    {
        this.driveTolerance = driveTolerance;
        this.thetaTolerance = thetaTolerance;
        this.driveController.setTolerance(driveTolerance);
        this.thetaController.setTolerance(thetaTolerance.getRadians());
        return this;
    }

    public DriveToPose finishWithinTolerance(boolean finish)
    {
        this.finishWithinTolerance = finish;
        return this;
    }

    @Override
    public void initialize()
    {
        Pose2d currentPose = robot.get();
        Pose2d targetPose = target.get();
        ChassisSpeeds fieldVelocity = drive.getChassisSpeeds();
        Translation2d linearFieldVelocity =
            new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);

        driveProfile =
            new TrapezoidProfile(new TrapezoidProfile.Constraints(
                driveMaxVelocity.get(), 4));

        this.driveController.setTolerance(driveTolerance);
        this.thetaController.setTolerance(thetaTolerance.getRadians());
        this.driveController.reset();
        this.thetaController.reset(
            currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
        lastSetpointTranslation = currentPose.getTranslation();
        lastSetpointVelocity = linearFieldVelocity;
        lastGoalRotation = targetPose.getRotation();
        lastTime = Timer.getTimestamp();
    }

    @Override
    public void execute()
    {
        running = true;

        // Update constraints
        double extensionS =
            MathUtil.clamp(
                RobotState.getInstance().getElevatorHeight() / 5.4,
                0.0,
                1.0);
        driveProfile =
            new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                    MathUtil.interpolate(
                        driveMaxVelocity.get(), driveMaxVelocityTop.get(), extensionS),
                    MathUtil.interpolate(
                        driveMaxAcceleration.get(), driveMaxAccelerationTop.get(),
                        extensionS)));
        thetaController.setConstraints(
            new TrapezoidProfile.Constraints(
                MathUtil.interpolate(
                    thetaMaxVelocity.get(), thetaMaxVelocityTop.get(), extensionS),
                MathUtil.interpolate(
                    thetaMaxAcceleration.get(), thetaMaxAccelerationTop.get(), extensionS)));

        // Get current pose and target pose
        Pose2d currentPose = robot.get();
        Pose2d targetPose = target.get();

        Pose2d poseError = currentPose.relativeTo(targetPose);
        driveErrorAbs = poseError.getTranslation().getNorm();
        thetaErrorAbs = Math.abs(poseError.getRotation().getRadians());
        double linearFFScaler =
            MathUtil.clamp(
                (driveErrorAbs - 0.01) / 0.04,
                0.0,
                1.0);
        double thetaFFScaler =
            MathUtil.clamp(
                Units.radiansToDegrees(thetaErrorAbs) / 0,
                0.0,
                1.0);

        // Calculate drive velocity
        // Calculate setpoint velocity towards target pose
        var direction = targetPose.getTranslation().minus(lastSetpointTranslation).toVector();
        double setpointVelocity =
            direction.norm() <= 0.01 // Don't calculate velocity in direction when really close
                ? lastSetpointVelocity.getNorm()
                : Math.abs(lastSetpointVelocity.toVector().dot(direction)) / direction.norm();

        State driveSetpoint =
            driveProfile.calculate(
                Constants.loopPeriodSecs,
                new State(
                    direction.norm(), -setpointVelocity), // Use negative as profile has zero at
                                                          // target
                new State(0.0, 0.0));

        double driveVelocityScalar =
            driveController.calculate(driveErrorAbs, driveSetpoint.position)
                + driveSetpoint.velocity * linearFFScaler;
        if (driveErrorAbs < driveController.getErrorTolerance())
            driveVelocityScalar = 0.0;

        Rotation2d targetToCurrentAngle =
            currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle();
        Translation2d driveVelocity = new Translation2d(driveVelocityScalar, targetToCurrentAngle);
        lastSetpointTranslation =
            new Pose2d(targetPose.getTranslation(), targetToCurrentAngle)
                .transformBy(new Transform2d(driveSetpoint.position, 0.0, Rotation2d.kZero))
                .getTranslation();
        lastSetpointVelocity = new Translation2d(driveSetpoint.velocity, targetToCurrentAngle);

        // Calculate theta speed
        double thetaSetpointVelocity =
            Math.abs((targetPose.getRotation().minus(lastGoalRotation)).getDegrees()) < 10.0
                ? (targetPose.getRotation().minus(lastGoalRotation)).getRadians()
                    / (Timer.getTimestamp() - lastTime)
                : thetaController.getSetpoint().velocity;
        double thetaVelocity =
            thetaController.calculate(
                currentPose.getRotation().getRadians(),
                new State(targetPose.getRotation().getRadians(), thetaSetpointVelocity))
                + thetaController.getSetpoint().velocity * thetaFFScaler;
        if (thetaErrorAbs < thetaController.getPositionTolerance())
            thetaVelocity = 0.0;
        lastGoalRotation = targetPose.getRotation();
        lastTime = Timer.getTimestamp();

        // Add input ff
        driveVelocity =
            driveVelocity.plus(linearFF.get().times(driveMaxVelocity.get()));
        thetaVelocity = thetaVelocity + omegaFF.getAsDouble() * 2.7;

        // Command speeds
        drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                driveVelocity.getX(), driveVelocity.getY(), thetaVelocity,
                currentPose.getRotation()));

        // Log data
        Logger.recordOutput("DriveToPose/DistanceMeasured", driveErrorAbs);
        Logger.recordOutput("DriveToPose/DistanceSetpoint", driveSetpoint.position);
        Logger.recordOutput("DriveToPose/VelocitySetpoint", driveSetpoint.velocity);
        Logger.recordOutput("DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
        Logger.recordOutput("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
        Logger.recordOutput(
            "DriveToPose/Setpoint",
            new Pose2d[] {
                    new Pose2d(
                        lastSetpointTranslation,
                        Rotation2d.fromRadians(thetaController.getSetpoint().position))
            });
        Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {targetPose});
    }

    /** Checks if the robot pose is within the allowed drive and theta tolerances. */
    public boolean withinTolerance()
    {
        return running
            && Math.abs(driveErrorAbs) <= driveTolerance
            && Math.abs(thetaErrorAbs) <= thetaTolerance.getRadians();
    }

    public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance)
    {
        return running
            && Math.abs(driveErrorAbs) <= driveTolerance
            && Math.abs(thetaErrorAbs) <= thetaTolerance.getRadians();
    }

    @Override
    public boolean isFinished()
    {
        return finishWithinTolerance && withinTolerance();
    }

    @Override
    public void end(boolean interrupted)
    {
        drive.stop();
        running = false;
        // Clear logs
        Logger.recordOutput("DriveToPose/Setpoint", new Pose2d[] {});
        Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {});
    }
}
