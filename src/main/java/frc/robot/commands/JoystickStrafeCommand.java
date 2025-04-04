package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.Drive;

public class JoystickStrafeCommand extends DriveToPose {

    public JoystickStrafeCommand(Drive drive, DoubleSupplier xSupplier, Supplier<Pose2d> target)
    {
        super(
            drive,
            () -> target.get().transformBy(
                new Transform2d(0.0, drive.getPose().relativeTo(target.get()).getY(),
                    Rotation2d.k180deg)),
            drive::getPose,
            () -> new Translation2d(0.0, -xSupplier.getAsDouble())
                .rotateBy(target.get().getRotation()),
            () -> 0.0);
        super.resetProfilesOnLargeFF(false);
    }
}
