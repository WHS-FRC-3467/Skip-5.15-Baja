package frc.robot.util.PPCalcEndpoint;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.ReefSide;
import org.littletonrobotics.junction.Logger;

public class PPCalcEndpoint {
    public void calculateEndPose()
    {
        Translation2d offset = new Translation2d(Units.inchesToMeters(21), 0);
        System.out.println("AM I WORKING");
        for (int i = 0; i < 12; i++) {
            Pose2d test = FieldConstants.Reef.branchPositions.get(i)
                .get(FieldConstants.ReefHeight.L1).toPose2d();

            test =
                test.plus(new Transform2d(offset, Rotation2d.k180deg));
            // test.plus(new Transform2d(offset, test.getRotation()));
            double reefX = test.getX();
            Logger.recordOutput("PPCalcEndpoint/reefX" + i, reefX);
            double reefY = test.getY();
            double reefDegree = test.getRotation().getDegrees();
            Logger.recordOutput("PPCalcEndpoint/reefY" + i, reefY);
            Logger.recordOutput("PPCalcEndpoint/test" + i, test);
            Logger.recordOutput("PPCalcEndpoint/reefDegree" + i, reefDegree);
        }
    }
}
