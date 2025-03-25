// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LED;

import org.littletonrobotics.junction.AutoLog;

public interface LEDSubsystemIO {

    /*
     * Robot LED States
     */
    public static enum LEDState {
        DISABLED,
        DISABLED_TRANSLATION_OK,
        DISABLED_ROTATION_OK,
        DISABLED_BOTH_OK,
        AUTONOMOUS,
        INTAKING,
        CLIMBING,
        CLIMBED,
        SUPER_MOVE,
        ALIGNING,
        HAVE_CORAL,
        ENABLED,
        NOT_SET
    }

    // Game Piece Mode
    public static enum GPMode {
        CORAL,
        ALGAE,
        PROCESSOR,
        NOT_SET
    }

    // Alliance Color
    public static enum AllianceColor {
        RED,
        BLUE,
        UNDETERMINED
    }

    // Match Timer State
    public static enum MatchTimerState {
        OFF,
        TELEOP_BEGIN,
        LAST_60,
        LAST_20,
        LAST_10,
        END
    }

    @AutoLog
    abstract class LEDSubsystemIOInputs {
        public AllianceColor alliance;
        public LEDState ledState;
        public GPMode gpMode;
        public MatchTimerState matchTime;
        public String GamePiece;
        public String RobotState;
    }

    default void updateInputs(LEDSubsystemIOInputs inputs)
    {}

    /** Update Alliance Color */
    default void setAlliance(AllianceColor alliance)
    {}

    /** Update Game Piece mode indicator */
    default void setGPMode(GPMode newGPMode)
    {}

    /** Update Robot State indicator */
    default void setRobotState(LEDState newState)
    {}

    /** Update Match Timer */
    default void setMatchTimerState(MatchTimerState mtState)
    {}
}

