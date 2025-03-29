package frc.robot.util.PPCalcEndpoint;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.ReefHeight;
import frc.robot.util.Util;

public class PPCalcEndpoint {

    // Branch names in order of FieldConstants branch instantiation
    private List<String> branchNames =
        List.of("B", "A", "L", "K", "J", "I", "H", "G", "F", "E", "D", "C");

    public void calculatePPEndpoints()
    {
        double offset = Units.inchesToMeters(19);

        for (int i = 0; i < 12; i++) {
            Pose2d original =
                FieldConstants.Reef.branchPositions.get(i).get(ReefHeight.L1).toPose2d();
            Pose2d alignPosition = Util.moveForward(original, offset)
                .transformBy(new Transform2d(Translation2d.kZero, Rotation2d.k180deg));

            Logger.recordOutput("AlignPositions/" + branchNames.get(i), alignPosition);
        }
    }
}
