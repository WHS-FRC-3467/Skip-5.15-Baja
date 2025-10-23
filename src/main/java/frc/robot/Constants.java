// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {

    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
    public static double loopPeriodSecs = 0.02;

    // Use LoggedTunableNumbers
    public static final boolean tuningMode = true;


    public static RobotType getRobot()
    {
        return robotType;
    }

    public static double bumperWidth = 0.99;

    /**
     * This enum defines the runtime mode used by AdvantageKit. The mode is always "real" when
     * running on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and
     * "replay" (log replay from a file).
     */


    public static enum Mode {
        /** Running on a real robot. */
        REAL,
        /** Running a physics simulator. */
        SIM,
        /** Replaying from a log file. */
        REPLAY
    }



    public static class RobotConstants {
        public static String serial;
        public static boolean isComp;
        public static boolean isAlpha;

        public static final String compSerial = "0323CA19";
        public static final String alphaSerial = "03223885";
        static {
            if (Robot.isReal()) {
                serial = System.getenv("serialnum");
            } else {
                serial = "3467";
            }


            RobotConstants.isComp = serial.startsWith(RobotConstants.compSerial);
            RobotConstants.isAlpha = serial.startsWith(RobotConstants.alphaSerial);
        }
    }

    public static RobotType robotType = RobotConstants.isComp ? RobotType.BAJA
        : RobotConstants.isAlpha ? RobotType.GORT : RobotType.NONE;

    public enum RobotType {
        BAJA,
        GORT,
        NONE
    }

}


