// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;
import lombok.Getter;
import lombok.Setter;

public class RobotState {
    private static RobotState instance;

    // public enum TARGET {
    // NONE,
    // CORAL_STATION,
    // REEF,
    // PROCESSOR,
    // BARGE,
    // CAGES,
    // GROUND_ALGAE;
    // }

    // @Getter
    // private TARGET currentTarget = TARGET.NONE;

    // public enum GAMEPIECE {
    // NONE,
    // CORAL,
    // LOWER_ALGAE,
    // UPPER_ALGAE;
    // }

    // @Getter
    // @Setter
    // private GAMEPIECE heldGamepiece = GAMEPIECE.NONE;

    @Getter
    @Setter
    @AutoLogOutput(key = "RobotState/ElevatorHeight")
    private double elevatorHeight = 0.0;

    public static RobotState getInstance()
    {
        if (instance == null)
            instance = new RobotState();
        return instance;
    }

    // public Command setTargetCommand(TARGET newTarget)
    // {
    // return Commands.runOnce(() -> this.currentTarget = newTarget);
    // }


}
