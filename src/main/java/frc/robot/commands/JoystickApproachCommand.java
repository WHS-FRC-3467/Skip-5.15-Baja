package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.Drive;

public class JoystickApproachCommand extends DriveToPose {

    public JoystickApproachCommand(Drive drive, DoubleSupplier ySupplier, Supplier<Pose2d> target)
    {
        super(
            drive,
            () -> target.get().transformBy(
                new Transform2d(drive.getPose().relativeTo(target.get()).getX(), 0.0,
                    Rotation2d.k180deg)),
            drive::getPose);
        super.withFeedforward(() -> new Translation2d(ySupplier.getAsDouble(), 0.0)
            .rotateBy(target.get().getRotation()),
            () -> 0.0);
    }
}
