package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.Drive;

/**
 * A specialized DriveToPose command that allows the robot to approach a target pose using joystick
 * input for forward/backward movement.
 */
public class JoystickApproachCommand extends DriveToPose {

    /**
     * Constructs a JoystickApproachCommand.
     *
     * @param drive The drive subsystem controlling the robot's movement.
     * @param ySupplier A supplier for the joystick Y-axis input (typically forward/backward).
     * @param target A supplier providing the target Pose2d the robot should approach.
     */
    public JoystickApproachCommand(Drive drive, DoubleSupplier ySupplier, Supplier<Pose2d> target)
    {
        super(
            drive,
            // Transform the target pose to face toward the robot while approaching
            () -> target.get().transformBy(
                new Transform2d(drive.getPose().relativeTo(target.get()).getX(), 0.0,
                    Rotation2d.k180deg)),
            drive::getPose);

        // Add joystick-based linear feedforward in the direction of the target's rotation
        super.withFeedforward(
            () -> new Translation2d(ySupplier.getAsDouble(), 0.0)
                .rotateBy(target.get().getRotation()),
            () -> 0.0)

                // Do not finish based on tolerance; this command typically runs until manually
                // stopped
                .finishWithinTolerance(false);
    }
}
