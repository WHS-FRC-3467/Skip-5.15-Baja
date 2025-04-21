package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Drive.Drive;

/**
 * A specialized DriveToPose command that allows the robot to strafe (move side-to-side) toward a
 * target pose using joystick input.
 */
public class JoystickStrafeCommand extends DriveToPose {

    /**
     * Constructs a JoystickStrafeCommand.
     *
     * @param drive The drive subsystem responsible for movement.
     * @param xSupplier A supplier for the joystick X-axis input (typically left/right strafe).
     * @param target A supplier providing the target Pose2d the robot should strafe toward.
     */
    public JoystickStrafeCommand(Drive drive, DoubleSupplier xSupplier, Supplier<Pose2d> target)
    {
        super(
            drive,
            // Transform the target pose to face toward the robot and align laterally
            () -> target.get().transformBy(
                new Transform2d(0.0, drive.getPose().relativeTo(target.get()).getY(),
                    Rotation2d.k180deg)),
            drive::getPose);

        // Add joystick-based lateral feedforward in the direction of the target's rotation
        super.withFeedforward(
            () -> new Translation2d(0.0, -xSupplier.getAsDouble())
                .rotateBy(target.get().getRotation()),
            () -> 0.0)

                // Do not finish automatically; this command typically runs while held
                .finishWithinTolerance(false);
    }
}
