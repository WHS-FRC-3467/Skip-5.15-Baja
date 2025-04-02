// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
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
    DoubleSupplier xSupplier;
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

    TuneableProfiledPID alignController =
        new TuneableProfiledPID(
            "alignController",
            6,
            0.0,
            0,
            3.7,
            4);

    public JoystickStrafeCommand(
        Drive drive,
        DoubleSupplier xSupplier,
        Supplier<Pose2d> targetSupplier)
    {
        this.drive = drive;
        this.xSupplier = xSupplier;
        this.targetSupplier = targetSupplier;

        angleController.enableContinuousInput(-Math.PI, Math.PI);
        alignController.setGoal(0);

        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        targetPose2d = targetSupplier.get();
        alignController.reset(0);
        angleController.reset(drive.getPose().getRotation().getRadians());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        angleController.updatePID();
        alignController.updatePID();

        running = true;
        relativePose2d = drive.getPose().relativeTo(targetPose2d);
        targetRotation2d = targetPose2d.getRotation();

        // Calculate lateral linear velocity
        Translation2d offsetVector =
            new Translation2d(alignController.calculate(relativePose2d.getX()), 0);

        // Calculate total linear velocity
        Translation2d linearVelocity =
            getLinearVelocityFromJoysticks(0, -xSupplier.getAsDouble())
                .times(drive.getMaxLinearSpeedMetersPerSec())
                .plus(offsetVector)
                .rotateBy(targetRotation2d);

        // Calculate angular speed
        double omega =
            angleController.calculate(
                drive.getRotation().getRadians(),
                targetRotation2d.rotateBy(Rotation2d.k180deg).getRadians());

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
        running = false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }

    // Returns true when withing a lateral tolerance
    public boolean withinTolerance(double dist)
    {
        return running ? Math.abs(relativePose2d.getX()) < dist : false;
    }

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y)
    {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();
    }
}
