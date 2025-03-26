// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import lombok.Getter;

public class RobotState {


    public enum TARGET {
        NONE,
        CORAL_STATION,
        REEF,
        PROCESSOR,
        BARGE,
        CAGES,
        GROUND_ALGAE;
    }

    @Getter
    private TARGET target = TARGET.NONE;

    public enum GAMEPIECE {
        NONE,
        CORAL,
        LOWER_ALGAE,
        UPPER_ALGAE;
    }

    @Getter
    private GAMEPIECE gamepiece = GAMEPIECE.NONE;

}
