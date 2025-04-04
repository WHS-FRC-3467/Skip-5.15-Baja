// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

    TuneableProfiledPID yController =
        new TuneableProfiledPID(
            "yController",
            5.3,
            0.0,
            0.2,
            3.7,
            4);

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

        Logger.recordOutput("AutoAlign/DriveToPose/Target", targetPose2d);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        angleController.updatePID();
        xController.updatePID();
        yController.updatePID();

        running = true;
        relativePose2d = drive.getPose().relativeTo(targetPose2d);
        targetRotation2d = targetPose2d.getRotation();

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
