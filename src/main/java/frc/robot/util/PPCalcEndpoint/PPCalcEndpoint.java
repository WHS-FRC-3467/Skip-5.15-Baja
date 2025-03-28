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
        System.out.println("AM I WORKING");
        for (int i = 0; i < 6; i++) {
            Pose2d test = FieldConstants.Reef.branchPositions
                .get(List.of(FieldConstants.Reef.centerFaces).indexOf(i))
                .get(FieldConstants.ReefHeight.L1).toPose2d();
            double reefX = test.getX();
            Logger.recordOutput("PPCalcEndpoint/reefX" + i, reefX);
            double reefY = test.getY();
            Logger.recordOutput("PPCalcEndpoint/reefY" + i, reefY);
        }
    }
}
