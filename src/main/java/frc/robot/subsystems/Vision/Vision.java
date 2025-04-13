package frc.robot.subsystems.Vision;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Vision.AprilTagVision.AprilTagVision;
import frc.robot.subsystems.Vision.AprilTagVision.AprilTagVisionIO;
import frc.robot.subsystems.Vision.ObjectVision.ObjectVision;
import frc.robot.subsystems.Vision.ObjectVision.ObjectVisionIO;
import frc.robot.subsystems.Vision.ObjectVision.ObjectVisionConstants.ObjectType;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;

public class Vision extends SubsystemBase {
    private final AprilTagVision aprilTagVision;
    private final ObjectVision objectVision;

    private final Supplier<Pose2d> robot;

    private final LoggedTunableNumber sawAprilTagDebounceTime =
        new LoggedTunableNumber("SawAprilTagDebouncer", 15);

    private final Debouncer sawAprilTagDebouncer =
        new Debouncer(sawAprilTagDebounceTime.get(), DebounceType.kFalling);

    public final Trigger accuratePose;

    public Vision(Drive drive, AprilTagVisionIO[] aprilTagIO, ObjectVisionIO[] objectIO)
    {
        robot = drive::getPose;

        aprilTagVision = new AprilTagVision(drive, aprilTagIO);
        objectVision = new ObjectVision(robot, objectIO);

        accuratePose =
            new Trigger(() -> sawAprilTagDebouncer.calculate(aprilTagVision.visionHasTarget));
    }

    // No Object Detection
    public Vision(Drive drive, AprilTagVisionIO[] aprilTagIO)
    {
        this(drive, aprilTagIO, new ObjectVisionIO[] {new ObjectVisionIO() {}});
    }

    // No AprilTag Vision
    public Vision(Drive drive, ObjectVisionIO[] objectIO)
    {
        this(drive, new AprilTagVisionIO[] {new AprilTagVisionIO() {}}, objectIO);
    }

    // Fake IO for replays
    public Vision(Drive drive)
    {
        this(drive, new AprilTagVisionIO[] {new AprilTagVisionIO() {}});
    }

    @Override
    public void periodic()
    {
        // Run composed periodics
        aprilTagVision.periodic();
        objectVision.periodic();

        // Update based on tunable numbers
        if (sawAprilTagDebounceTime.hasChanged(hashCode())) {
            sawAprilTagDebouncer.setDebounceTime(sawAprilTagDebounceTime.get());
        }
    }

    public List<Pose2d> getSeenObjects(ObjectType objectType)
    {
        return objectVision.getObjectPoses().get(objectType);
    }

    public Optional<Pose2d> getNearestObject(ObjectType objectType)
    {
        return Optional.ofNullable(robot.get().nearest(getSeenObjects(objectType)));
    }
}
