// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;
import lombok.Getter;
import lombok.Setter;

public class RobotState {
    private static RobotState instance;

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
}
