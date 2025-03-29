// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public class Constants {

    public static double loopPeriodSecs = 0.02;

    // Use LoggedTunableNumbers
    public static final boolean tuningMode = true;

    private static RobotType robotType = RobotType.GORT;

    public static RobotType getRobot()
    {
        return robotType;
    }

    /**
     * This enum defines the runtime mode used by AdvantageKit. The mode is always "real" when
     * running on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and
     * "replay" (log replay from a file).
     */
    public static final Mode simMode = Mode.SIM;

    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,
        /** Running a physics simulator. */
        SIM,
        /** Replaying from a log file. */
        REPLAY
    }

    public enum RobotType {
        GORT,
        BAJA
    }
}


