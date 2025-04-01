// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.TuneableProfiledPID;

public class DriveToPose extends Command {
    Drive drive;
    double xTolerance;
    double yTolerance;
    double rotationTolerance;
    Supplier<Pose2d> targetSupplier;

    Pose2d targetPose2d;
    Pose2d currentPose2d;
    Pose2d relativePose2d;
    Rotation2d targetRotation2d;

    boolean running = false;

    static final double DEADBAND = 0.1;

    TuneableProfiledPID angleController =
        new TuneableProfiledPID(
            "angleController",
            5.0,
            0.0,
            0.4,
            8.0,
            20.0);

    TuneableProfiledPID xController =
        new TuneableProfiledPID(
            "xController",
            2,
            0.0,
            0.1,
            20,
            8);

    TuneableProfiledPID yController =
        new TuneableProfiledPID(
            "yController",
            2,
            0.0,
            0.1,
            20,
            8);

    public DriveToPose(
        Drive drive,
        Supplier<Pose2d> targetSupplier,
        double xTolerance,
        double yTolerance,
        double rotationTolerance)
    {
        this.drive = drive;
        this.targetSupplier = targetSupplier;
        this.xTolerance = xTolerance;
        this.yTolerance = yTolerance;
        this.rotationTolerance = rotationTolerance;

        angleController.enableContinuousInput(-Math.PI, Math.PI);
        xController.setGoal(0);
        yController.setGoal(0);

        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        targetPose2d = targetSupplier.get();
        relativePose2d = drive.getPose().relativeTo(targetPose2d);
        targetRotation2d = targetSupplier.get().getRotation();
        xController.reset(relativePose2d.getX());
        yController.reset(relativePose2d.getY());
        angleController.reset(drive.getPose().getRotation().getRadians());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        angleController.updatePID();
        xController.updatePID();
        yController.updatePID();

        running = true;
        targetPose2d = targetSupplier.get();
        relativePose2d = drive.getPose().relativeTo(targetPose2d);
        targetRotation2d = targetSupplier.get().getRotation();

        Logger.recordOutput("test", relativePose2d);

        // Calculate lateral linear velocity
        Translation2d offsetVector =
            new Translation2d(xController.calculate(relativePose2d.getX()),
                yController.calculate(relativePose2d.getY()));

        // Calculate total linear velocity
        Translation2d linearVelocity = offsetVector
            .rotateBy(targetRotation2d);

        // Calculate angular speed
        double omega =
            angleController.calculate(
                drive.getRotation().getRadians(),
                targetRotation2d.getRadians());

        // Convert to field relative speeds & send command
        ChassisSpeeds speeds =
            new ChassisSpeeds(
                MathUtil.clamp(linearVelocity.getX(), 0, drive.getMaxLinearSpeedMetersPerSec()),
                MathUtil.clamp(linearVelocity.getY(), 0, drive.getMaxLinearSpeedMetersPerSec()),
                MathUtil.clamp(omega, 0, drive.getMaxAngularSpeedRadPerSec()));

        drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                drive.getRotation()));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        drive.stopWithX();
        running = false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return running
            && (Math.abs(relativePose2d.getX()) < xTolerance)
            && (Math.abs(relativePose2d.getY()) < yTolerance)
            && (Math.abs(relativePose2d.getRotation().getRotations()) < rotationTolerance);
    }
}
