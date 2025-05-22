// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.TuneableProfiledPID;

public class JoystickStrafeCommand extends Command {
    Drive drive;
    double driveTolerance = 0.0;
    Rotation2d rotationTolerance = Rotation2d.kZero;
    Supplier<Pose2d> targetSupplier;
    DoubleSupplier joystickSupplier;

    boolean finishWithinTolerance = false;

    Pose2d targetPose2d;
    Pose2d currentPose2d;
    Pose2d relativePose2d;
    Rotation2d targetRotation2d;

    boolean running = false;

    static final double DEADBAND = 0.1;

    TuneableProfiledPID angleController =
        new TuneableProfiledPID(
            "angleController",
            4.5,
            0.0,
            0.4,
            20,
            20.0);

    TuneableProfiledPID xController =
        new TuneableProfiledPID(
            "xController",
            5.3,
            0.0,
            0.2,
            3.7,
            4);

    public JoystickStrafeCommand(
        Drive drive,
        DoubleSupplier joystickSupplier,
        Supplier<Pose2d> targetSupplier)
    {
        this.drive = drive;
        this.targetSupplier = targetSupplier;
        this.joystickSupplier = joystickSupplier;

        angleController.enableContinuousInput(-Math.PI, Math.PI);
        xController.setGoal(0);

        addRequirements(drive);
    }

    public JoystickStrafeCommand withTolerance(double driveTolerance, Rotation2d thetaTolerance)
    {
        this.driveTolerance = driveTolerance;
        this.rotationTolerance = thetaTolerance;
        return this;
    }

    public JoystickStrafeCommand finishWithinTolerance(boolean finish)
    {
        this.finishWithinTolerance = finish;
        return this;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        targetPose2d = targetSupplier.get().transformBy(new Transform2d(0, 0, Rotation2d.k180deg));
        relativePose2d = drive.getPose().relativeTo(targetPose2d);
        targetRotation2d = targetSupplier.get().getRotation();
        xController.reset(relativePose2d.getX());
        angleController.reset(drive.getPose().getRotation().getRadians());

        Logger.recordOutput("AutoAlign/Strafe/Target", targetPose2d);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        angleController.updatePID();
        xController.updatePID();

        running = true;
        relativePose2d = drive.getPose().relativeTo(targetPose2d);
        targetRotation2d = targetPose2d.getRotation();

        // Calculate lateral linear velocity
        Translation2d offsetVector =
            new Translation2d(xController.calculate(relativePose2d.getX()), 0.0);

        // Calculate total linear velocity
        Translation2d linearVelocity = offsetVector
            .plus(DriveCommands.getLinearVelocityFromJoysticks(0.0, joystickSupplier.getAsDouble()))
            .rotateBy(targetRotation2d);

        // Calculate angular speed
        double omega =
            angleController.calculate(
                drive.getRotation().getRadians(),
                targetRotation2d.getRadians());

        // Convert to field relative speeds & send command
        ChassisSpeeds speeds =
            new ChassisSpeeds(
                linearVelocity.getX(),
                linearVelocity.getY(),
                omega);

        drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                drive.getRotation()));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        drive.stop();
        running = false;
    }

    public boolean withinTolerance(double driveTolerance, Rotation2d rotationTolerance)
    {
        return (Math.abs(relativePose2d.getX()) < driveTolerance)
            && (Math.abs(relativePose2d.getY()) < driveTolerance)
            && (Math.abs(relativePose2d.getRotation().getRotations()) < rotationTolerance
                .getRotations());
    }

    public boolean withinTolerance()
    {
        return (Math.abs(relativePose2d.getX()) < driveTolerance)
            && (Math.abs(relativePose2d.getY()) < driveTolerance)
            && (Math.abs(relativePose2d.getRotation().getRotations()) < rotationTolerance
                .getRotations());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return running && finishWithinTolerance && withinTolerance();
    }
}
