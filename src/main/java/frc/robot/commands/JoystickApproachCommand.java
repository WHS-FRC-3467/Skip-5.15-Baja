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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommands.DriveMode;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.TuneableProfiledPID;

public class JoystickApproachCommand extends Command {
    Drive drive;
    DoubleSupplier xSupplier;
    DoubleSupplier ySupplier;
    Supplier<Pose2d> targetSupplier;

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
            0.6,
            0.0,
            0,
            20,
            8);

    public JoystickApproachCommand(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
        Supplier<Pose2d> targetSupplier)
    {
        this.drive = drive;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.targetSupplier = targetSupplier;

        angleController.enableContinuousInput(-Math.PI, Math.PI);
        alignController.setGoal(0);

        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        Pose2d currentPose2d = drive.getPose();
        alignController.reset(0);
        angleController.reset(currentPose2d.getRotation().getRadians());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        Translation2d currentTranslation = drive.getPose().getTranslation();
        Translation2d approachTranslation = targetSupplier.get().getTranslation();
        double distanceToApproach = currentTranslation.getDistance(approachTranslation);

        Rotation2d alignmentDirection = targetSupplier.get().getRotation();

        // Find lateral distance from goal
        Translation2d goalTranslation = new Translation2d(
            alignmentDirection.getCos() * distanceToApproach + approachTranslation.getX(),
            alignmentDirection.getSin() * distanceToApproach + approachTranslation.getY());

        Translation2d robotToGoal = currentTranslation.minus(goalTranslation);
        double distanceToGoal =
            Math.hypot(robotToGoal.getX(), robotToGoal.getY());

        // Calculate lateral linear velocity
        Translation2d offsetVector =
            new Translation2d(alignController.calculate(distanceToGoal), 0)
                .rotateBy(robotToGoal.getAngle());

        // Calculate total linear velocity
        Translation2d linearVelocity =
            getLinearVelocityFromJoysticks(0,
                ySupplier.getAsDouble()).rotateBy(
                    targetSupplier.get().getRotation()).rotateBy(Rotation2d.kCCW_90deg)
                    .plus(offsetVector);

        // Calculate angular speed
        double omega =
            angleController.calculate(
                drive.getRotation().getRadians(), targetSupplier.get().getRotation()
                    .rotateBy(Rotation2d.k180deg).getRadians());

        // Convert to field relative speeds & send command
        ChassisSpeeds speeds =
            new ChassisSpeeds(
                linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                omega);

        drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                drive.getRotation()));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
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
