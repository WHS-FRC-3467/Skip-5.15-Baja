// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.FieldConstants.ReefSide;
import frc.robot.subsystems.Drive.Drive;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
    private static final double ANGLE_KP = 5.0;
    private static final double ANGLE_KD = 0.4;
    private static final double ANGLE_MAX_VELOCITY = 8.0;
    private static final double ANGLE_MAX_ACCELERATION = 20.0;
    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

    private static final RobotState robotState = RobotState.getInstance();

    private DriveCommands()
    {}

    public static Translation2d getLinearVelocityFromJoysticks(double x, double y)
    {
        double linearMagnitude = Math.hypot(x, y);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Translation2d(linearMagnitude, linearDirection);
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     */
    public static Command joystickDrive(
        Drive drive,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier,
        DoubleSupplier omegaSupplier)
    {
        return Commands.run(
            () -> {
                // Get linear velocity
                Translation2d linearVelocity =
                    getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
                        ySupplier.getAsDouble());

                double omega = omegaSupplier.getAsDouble();

                // Square rotation value for more precise control
                omega = Math.copySign(omega * omega, omega);

                // Convert to field relative speeds & send command
                ChassisSpeeds speeds =
                    new ChassisSpeeds(
                        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                        omega * drive.getMaxAngularSpeedRadPerSec());
                boolean isFlipped =
                    DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red;
                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        speeds,
                        isFlipped
                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                            : drive.getRotation()));
            },
            drive).withName("Drivetrain: Joystick Drive");
    }

    /**
     * Field relative drive command using joystick for linear control and PID for angular control.
     * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
     * absolute rotation with a joystick.
     */
    public static Command joystickDriveAtAngle(
        Drive drive,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier,
        Supplier<Rotation2d> rotationSupplier)
    {

        // Create PID controller
        ProfiledPIDController angleController =
            new ProfiledPIDController(
                ANGLE_KP,
                0.0,
                ANGLE_KD,
                new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        // Construct command
        return Commands.run(
            () -> {
                // Get linear velocity
                Translation2d linearVelocity =
                    getLinearVelocityFromJoysticks(
                        xSupplier.getAsDouble(),
                        ySupplier.getAsDouble());

                // Calculate angular speed
                double omega =
                    angleController.calculate(
                        drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

                // Convert to field relative speeds & send command
                ChassisSpeeds speeds =
                    new ChassisSpeeds(
                        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                        omega);
                boolean isFlipped =
                    DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red;
                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        speeds,
                        isFlipped
                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                            : drive.getRotation()));
            },
            drive)

            // Reset PID controller when command starts
            .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()))
            .withName("Drivetrain: Drive At Angle");
    }

    /**
     * Creates a {@link DriveToPose} command that drives to the nearest reef branch on the specified
     * side, with an optional prediction into the future.
     *
     * @param drive The drive subsystem.
     * @param side The side of the reef to approach (LEFT or RIGHT).
     * @param predictionSeconds Time in seconds to predict ahead when determining the reef branch
     *        position.
     * @return A DriveToPose command targeting the predicted reef branch pose.
     */
    public static DriveToPose driveToReefBranch(Drive drive, ReefSide side,
        DoubleSupplier predictionSeconds)
    {
        return new DriveToPose(drive,
            () -> robotState.getNearestReefBranch(side, predictionSeconds.getAsDouble())
                .transformBy(new Transform2d(Constants.bumperWidth / 2 + Units.inchesToMeters(1),
                    0.0, Rotation2d.k180deg)));
    }

    /**
     * Overload of {@link #driveToReefBranch(Drive, ReefSide, DoubleSupplier)} with zero prediction
     * time.
     *
     * @param drive The drive subsystem.
     * @param side The side of the reef to approach (LEFT or RIGHT).
     * @return A DriveToPose command targeting the nearest reef branch.
     */
    public static DriveToPose driveToReefBranch(Drive drive, ReefSide side)
    {
        return driveToReefBranch(drive, side, () -> 0.0);
    }

    /**
     * Creates a {@link DriveToPose} command that drives to the nearest reef face, with an optional
     * prediction into the future.
     *
     * @param drive The drive subsystem.
     * @param predictionSeconds Time in seconds to predict ahead when determining the reef face
     *        position.
     * @return A DriveToPose command targeting the predicted reef face pose.
     */
    public static DriveToPose driveToReefFace(Drive drive, DoubleSupplier predictionSeconds)
    {
        return new DriveToPose(drive,
            () -> robotState.getNearestReefFace(predictionSeconds.getAsDouble())
                .transformBy(new Transform2d(Constants.bumperWidth / 2 + Units.inchesToMeters(1),
                    0.0, Rotation2d.k180deg)));
    }

    /**
     * Overload of {@link #driveToReefFace(Drive, DoubleSupplier)} with zero prediction time.
     *
     * @param drive The drive subsystem.
     * @return A DriveToPose command targeting the nearest reef face.
     */
    public static DriveToPose driveToReefFace(Drive drive)
    {
        return driveToReefFace(drive, () -> 0.0);
    }

    /**
     * Creates a {@link JoystickApproachCommand} that allows manual joystick-controlled approach to
     * the nearest reef branch on the given side, with pose prediction.
     *
     * @param drive The drive subsystem.
     * @param ySupplier Supplies the joystick Y input.
     * @param side The side of the reef to approach (LEFT or RIGHT).
     * @param predictionSeconds Time in seconds to predict ahead when determining the reef branch
     *        position.
     * @return A JoystickApproachCommand for approaching the reef branch.
     */
    public static JoystickApproachCommand approachReefBranch(Drive drive, DoubleSupplier ySupplier,
        ReefSide side, DoubleSupplier predictionSeconds)
    {
        return new JoystickApproachCommand(drive, ySupplier,
            () -> robotState.getNearestReefBranch(side, predictionSeconds.getAsDouble()));
    }

    /**
     * Overload of {@link #approachReefBranch(Drive, DoubleSupplier, ReefSide, DoubleSupplier)} with
     * zero prediction time.
     *
     * @param drive The drive subsystem.
     * @param ySupplier Supplies the joystick Y input.
     * @param side The side of the reef to approach (LEFT or RIGHT).
     * @return A JoystickApproachCommand for approaching the reef branch.
     */
    public static JoystickApproachCommand approachReefBranch(Drive drive, DoubleSupplier ySupplier,
        ReefSide side)
    {
        return approachReefBranch(drive, ySupplier, side, () -> 0.0);
    }

    /**
     * Creates a {@link JoystickApproachCommand} that allows manual joystick-controlled approach to
     * the nearest reef face, with pose prediction.
     *
     * @param drive The drive subsystem.
     * @param ySupplier Supplies the joystick Y input.
     * @param predictionSeconds Time in seconds to predict ahead when determining the reef face
     *        position.
     * @return A JoystickApproachCommand for approaching the reef face.
     */
    public static JoystickApproachCommand approachReefFace(Drive drive, DoubleSupplier ySupplier,
        DoubleSupplier predictionSeconds)
    {
        return new JoystickApproachCommand(drive,
            ySupplier,
            () -> robotState.getNearestReefFace(predictionSeconds.getAsDouble()));
    }

    /**
     * Overload of {@link #approachReefFace(Drive, DoubleSupplier, DoubleSupplier)} with zero
     * prediction time.
     *
     * @param drive The drive subsystem.
     * @param ySupplier Supplies the joystick Y input.
     * @return A JoystickApproachCommand for approaching the reef face.
     */
    public static DriveToPose approachReefFace(Drive drive, DoubleSupplier ySupplier)
    {
        return approachReefFace(drive, ySupplier, () -> 0.0);
    }

    /**
     * Creates a command that orients the robot to face the nearest reef face while allowing
     * joystick-based translational control.
     *
     * @param drive The drive subsystem used to control the robot.
     * @param xSupplier Supplies the joystick X input.
     * @param ySupplier Supplies the joystick Y input.
     * @param predictionSeconds A supplier for the amount of time into the future to predict reef
     *        face position (not used directly here but included for API consistency).
     * @return A command that drives the robot with joystick input while rotating it to face the
     *         reef.
     */
    public static Command faceReef(Drive drive, DoubleSupplier xSupplier,
        DoubleSupplier ySupplier, DoubleSupplier predictionSeconds)
    {
        return joystickDriveAtAngle(
            drive,
            xSupplier,
            ySupplier,
            // Calculate the angle 180° away from the nearest reef face rotation
            () -> robotState.getNearestReefFace(predictionSeconds.getAsDouble())
                .getRotation()
                .rotateBy(Rotation2d.k180deg));
    }

    /**
     * Overload of {@link #faceReef(Drive, DoubleSupplier, DoubleSupplier, DoubleSupplier)} with
     * zero prediction time.
     *
     * @param drive The drive subsystem.
     * @param xSupplier Supplies the joystick X input.
     * @param ySupplier Supplies the joystick Y input.
     * @return A command that drives the robot with joystick input while rotating it to face the
     *         reef.
     */
    public static Command faceReef(Drive drive, DoubleSupplier xSupplier,
        DoubleSupplier ySupplier)
    {
        return faceReef(drive, xSupplier, ySupplier, () -> 0.0);
    }

    /**
     * Creates a command that orients the robot to face the processor while allowing joystick-based
     * translational control.
     *
     * @param drive The drive subsystem used to control the robot.
     * @param xSupplier Supplies the joystick X input.
     * @param ySupplier Supplies the joystick Y input.
     * @param predictionSeconds A supplier for the amount of time into the future to predict reef
     *        face position (not used directly here but included for API consistency).
     * @return A command that drives the robot with joystick input while rotating it to face the
     *         nearest processor.
     */
    public static Command faceProcessor(Drive drive, DoubleSupplier xSupplier,
        DoubleSupplier ySupplier, DoubleSupplier predictionSeconds)
    {
        return joystickDriveAtAngle(
            drive,
            xSupplier,
            ySupplier,
            // Calculate the angle 180° away from the nearest processor rotation
            () -> robotState.getNearestProcessor(predictionSeconds.getAsDouble()).getRotation()
                .rotateBy(Rotation2d.k180deg));
    }

    /**
     * Overload of {@link #faceProcessor(Drive, DoubleSupplier, DoubleSupplier, DoubleSupplier)}
     * with zero prediction time.
     *
     * @param drive The drive subsystem.
     * @param xSupplier Supplies the joystick X input.
     * @param ySupplier Supplies the joystick Y input.
     * @return A command that drives the robot with joystick input while rotating it to face the
     *         reef.
     */
    public static Command faceProcessor(Drive drive, DoubleSupplier xSupplier,
        DoubleSupplier ySupplier)
    {
        return faceProcessor(drive, xSupplier, ySupplier, () -> 0.0);
    }

    /**
     * Creates a {@link JoystickApproachCommand} that allows manual joystick-controlled approach to
     * the nearest processor, with pose prediction.
     *
     * @param drive The drive subsystem.
     * @param ySupplier Supplies the joystick Y input.
     * @param predictionSeconds Time in seconds to predict ahead when determining the processor
     *        position.
     * @return A JoystickApproachCommand for approaching the reef branch.
     */
    public static JoystickApproachCommand approachProcessor(Drive drive, DoubleSupplier ySupplier,
        DoubleSupplier predictionSeconds)
    {
        return new JoystickApproachCommand(drive, ySupplier,
            () -> robotState.getNearestProcessor());
    }

    /**
     * Overload of {@link #approachProcessor(Drive, DoubleSupplier, DoubleSupplier)} with zero
     * prediction time.
     *
     * @param drive The drive subsystem.
     * @param ySupplier Supplies the joystick Y input.
     * @return A JoystickApproachCommand for approaching the reef branch.
     */
    public static JoystickApproachCommand approachProcessor(Drive drive, DoubleSupplier ySupplier)
    {
        return approachProcessor(drive, ySupplier, () -> 0.0);
    }

    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>
     * This command should only be used in voltage control mode.
     */
    public static Command feedforwardCharacterization(Drive drive)
    {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
            // Reset data
            Commands.runOnce(
                () -> {
                    velocitySamples.clear();
                    voltageSamples.clear();
                }),

            // Allow modules to orient
            Commands.run(
                () -> {
                    drive.runCharacterization(0.0);
                },
                drive)
                .withTimeout(FF_START_DELAY),

            // Start timer
            Commands.runOnce(timer::restart),

            // Accelerate and gather data
            Commands.run(
                () -> {
                    double voltage = timer.get() * FF_RAMP_RATE;
                    drive.runCharacterization(voltage);
                    velocitySamples.add(drive.getFFCharacterizationVelocity());
                    voltageSamples.add(voltage);
                },
                drive)

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                        int n = velocitySamples.size();
                        double sumX = 0.0;
                        double sumY = 0.0;
                        double sumXY = 0.0;
                        double sumX2 = 0.0;
                        for (int i = 0; i < n; i++) {
                            sumX += velocitySamples.get(i);
                            sumY += voltageSamples.get(i);
                            sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                            sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                        }
                        double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                        double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                        Logger.recordOutput("Drive/Characterization/kS", kS);
                        Logger.recordOutput("Drive/Characterization/kV", kV);
                    }));
    }



    /** Measures the robot's wheel radius by spinning in a circle. */
    public static Command wheelRadiusCharacterization(Drive drive)
    {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
            // Drive control sequence
            Commands.sequence(
                // Reset acceleration limiter
                Commands.runOnce(
                    () -> {
                        limiter.reset(0.0);
                    }),

                // Turn in place, accelerating up to full speed
                Commands.run(
                    () -> {
                        double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                        drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                    },
                    drive)),

            // Measurement sequence
            Commands.sequence(
                // Wait for modules to fully orient before starting measurement
                Commands.waitSeconds(1.0),

                // Record starting measurement
                Commands.runOnce(
                    () -> {
                        state.positions = drive.getWheelRadiusCharacterizationPositions();
                        state.lastAngle = drive.getRotation();
                        state.gyroDelta = 0.0;
                    }),

                // Update gyro delta
                Commands.run(
                    () -> {
                        var rotation = drive.getRotation();
                        state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                        state.lastAngle = rotation;
                    })

                    // When cancelled, calculate and print results
                    .finallyDo(
                        () -> {
                            double[] positions = drive.getWheelRadiusCharacterizationPositions();
                            double wheelDelta = 0.0;
                            for (int i = 0; i < 4; i++) {
                                wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                            }
                            double wheelRadius =
                                (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                            Logger.recordOutput("Drive/Characterization/WheelDelta",
                                Rotation2d.fromRadians(wheelDelta));
                            Logger.recordOutput("Drive/Characterization/GyroDelta",
                                Rotation2d.fromRadians(state.gyroDelta));
                            Logger.recordOutput("Drive/Characterization/WheelRadiusInches",
                                Units.metersToInches(wheelRadius));
                        })));
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = new Rotation2d();
        double gyroDelta = 0.0;
    }
}
