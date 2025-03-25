// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LED;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Ports;

public class LEDSubsystemIOCANdle implements LEDSubsystemIO {

    // Control everything with a CANdle
    private static final CANdle m_candle = new CANdle(Ports.ELEVATOR_CANDLE.getDeviceNumber());

    Alert ledConfigError = new Alert("CANdle Configuration Error!", Alert.AlertType.kWarning);

    LEDState m_currentState = LEDState.NOT_SET;
    GPMode m_currentGPMode = GPMode.CORAL;
    AllianceColor m_DSAlliance = AllianceColor.UNDETERMINED;
    Color m_allianceColor = Color.kBlack;
    MatchTimerState m_mtState = MatchTimerState.END;
    String m_tipColor = "x000000";
    String m_stateColor = "x000000";

    // LED Index Constants
    final int TOTAL_LEDS = 182;
    final int BASESEG_SIZE = 8;
    final int TIP_SIZE = 43;
    final int FULLSTRIP_SIZE = 87;
    final int MATCHTIME_START = 0;
    final int MATCHTIME_END = BASESEG_SIZE - 1;
    final int RIGHTSTRIP_START = MATCHTIME_END + 1;
    final int RIGHTSTRIP_END = RIGHTSTRIP_START + FULLSTRIP_SIZE - 1;
    final int LEFTSTRIP_START = RIGHTSTRIP_END + 1;
    final int LEFTSTRIP_END = LEFTSTRIP_START + FULLSTRIP_SIZE - 1;;
    final int RIGHTTIP_START = MATCHTIME_END + 1;
    final int RIGHTTIP_END = RIGHTTIP_START + TIP_SIZE - 1;
    final int STATE_START = RIGHTTIP_END + 1;
    final int LEFTTIP_START = TOTAL_LEDS - TIP_SIZE;
    final int LEFTTIP_END = TOTAL_LEDS - 1;
    final int STATE_END = LEFTTIP_START - 1;

    // Define LED Segments
    // Numbering Sequence: Right LEDs go down, Left LEDs go Up
    // Match Time Indicator - CANdle module
    LEDSegment m_MatchTime = new LEDSegment(MATCHTIME_START, BASESEG_SIZE, 0);
    // These are for Disabled/Auto states
    // They are each a full strip
    LEDSegment m_FullRight = new LEDSegment(RIGHTSTRIP_START, FULLSTRIP_SIZE, 1);
    LEDSegment m_FullLeft = new LEDSegment(LEFTSTRIP_START, FULLSTRIP_SIZE, 2);
    // These are for Coral/Algae Mode while robot is Enabled
    // These are the top pixels on each strip
    LEDSegment m_RightTip = new LEDSegment(RIGHTTIP_START, TIP_SIZE, 3);
    LEDSegment m_LeftTip = new LEDSegment(LEFTTIP_START, TIP_SIZE, 4);
    // This is for what the robot is doing while robot is Enabled
    // This is the bottom of each strip combined into one segment
    LEDSegment m_State = new LEDSegment(STATE_START, (STATE_END - STATE_START) + 1, 5);

    public LEDSubsystemIOCANdle()
    {
        m_candle.configFactoryDefault();

        CANdleConfiguration candleConfiguration = new CANdleConfiguration();
        // m_candle.getAllConfigs(candleConfiguration);

        candleConfiguration.statusLedOffWhenActive = true;
        candleConfiguration.disableWhenLOS = false;
        candleConfiguration.stripType = LEDStripType.RGB;
        candleConfiguration.brightnessScalar = 0.5;
        candleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;
        candleConfiguration.v5Enabled = false;
        ErrorCode ec = m_candle.configAllSettings(candleConfiguration, 100);
        if (ec != ErrorCode.OK) {
            ledConfigError.set(true);
            ledConfigError.setText(ec.toString());
        }

        // StripType setting needs to be done twice (for some reason, once doesn't work)
        ec = m_candle.configLEDType(LEDStripType.RGB, 300);
        if (ec != ErrorCode.OK) {
            ledConfigError.set(true);
            ledConfigError.setText(ec.toString());
        }
    }

    @Override
    public void updateInputs(LEDSubsystemIOInputs inputs)
    {
        inputs.alliance = m_DSAlliance;
        inputs.ledState = m_currentState;
        inputs.gpMode = m_currentGPMode;
        inputs.matchTime = m_mtState;
        inputs.GamePiece = m_tipColor;
        inputs.RobotState = m_stateColor;
    }

    /** Update Alliance Color */
    @Override
    public void setAlliance(AllianceColor alliance)
    {
        switch (alliance) {
            case RED:
                m_allianceColor = Color.kRed;
                break;
            case BLUE:
                m_allianceColor = Color.kBlue;
                break;
            case UNDETERMINED:
            default:
                m_allianceColor = Color.kMagenta;
                break;
        }
        m_DSAlliance = alliance;
    }

    // Set the Game Piece Mode indicator LEDS
    @Override
    public void setGPMode(GPMode newGPMode)
    {
        // Mode:
        // - GPMode -> Tips: White or "algae" color

        // Process and make changes for changed GPMode
        switch (m_currentState) {
            case DISABLED:
            case DISABLED_TRANSLATION_OK:
            case DISABLED_ROTATION_OK:
            case DISABLED_BOTH_OK:
            case AUTONOMOUS:
                // Mode is not displayed in these cases
                // so just break out
                newGPMode = GPMode.NOT_SET;
                m_tipColor = Color.kBlack.toHexString();
                break;
            default:
                if (newGPMode != m_currentGPMode) {
                    switch (newGPMode) {
                        case PROCESSOR:
                            m_LeftTip.setAnimation(a_FastFlashGreen);
                            m_RightTip.setAnimation(a_FastFlashGreen);
                            m_tipColor = Color.kAqua.toHexString();
                            break;
                        case ALGAE:
                            m_LeftTip.setColor(Color.kGreen);
                            m_RightTip.setColor(Color.kGreen);
                            m_tipColor = Color.kGreen.toHexString();
                            break;
                        case CORAL:
                        default:
                            m_LeftTip.setColor(Color.kWhite);
                            m_RightTip.setColor(Color.kWhite);
                            m_tipColor = Color.kWhite.toHexString();
                            break;
                    }
                    m_currentGPMode = newGPMode;
                }
                break;
        }
    }

    // Set the Robot State indicator LEDS
    @Override
    public void setRobotState(LEDState newState)
    {

        // --- Order of Priorities ---
        // Whole Robot:
        // - DISABLED -> Rainbow
        // - DISABLED_TRANSLATION_OK -> Left side Green
        // - DISABLED_ROTATION_OK -> Right side Green
        // - DISABLED_BOTH_OK -> Both sides Green
        // - AUTONOMOUS -> Flames
        // State:
        // - INTAKING -> Red Flash Slow
        // - FEEDING -> Blue
        // - CLIMBING -> Red Flash Fast
        // - CLIMBED -> Green
        // - SUPER_MOVE -> Magenta
        // - ALIGNING -> Cyan
        // - HAVE_CORAL -> Green
        // - ENABLED -> Yellow

        // Don't do anything unless state has changed
        if (m_currentState == newState) {
            return;
        }

        // Process and make changes for changed LEDState
        switch (newState) {
            case DISABLED:
                m_FullLeft.setColor(m_allianceColor);
                m_FullRight.setColor(m_allianceColor);
                m_tipColor = m_allianceColor.toHexString();
                m_stateColor = m_allianceColor.toHexString();
                break;

            case DISABLED_TRANSLATION_OK:
                m_FullLeft.setColor(Color.kGreen);
                m_FullRight.setColor(m_allianceColor);
                m_tipColor = Color.kGreen.toHexString();
                m_stateColor = m_allianceColor.toHexString();
                break;

            case DISABLED_ROTATION_OK:
                m_FullLeft.setColor(m_allianceColor);
                m_FullRight.setColor(Color.kGreen);
                m_tipColor = m_allianceColor.toHexString();
                m_stateColor = Color.kGreen.toHexString();
                break;

            case DISABLED_BOTH_OK:
                m_FullLeft.setColor(Color.kGreen);
                m_FullRight.setColor(Color.kGreen);
                m_tipColor = Color.kGreen.toHexString();
                m_stateColor = Color.kGreen.toHexString();
                break;

            case AUTONOMOUS:
                m_FullLeft.setAnimation(a_LeftFlame);
                m_FullRight.setAnimation(a_RightFlame);
                m_MatchTime.setAnimation(a_InAutonomous);
                m_tipColor = Color.kOrange.toHexString();
                m_stateColor = Color.kOrange.toHexString();
                break;

            case INTAKING:
                m_State.setAnimation(a_SlowFlashRed);
                m_stateColor = Color.kRed.toHexString();
                break;

            case CLIMBING:
                m_State.setAnimation(a_FastFlashRed);
                m_stateColor = Color.kRed.toHexString();
                break;

            case CLIMBED:
                m_State.setColor(Color.kGreen);
                m_stateColor = Color.kGreen.toHexString();
                break;

            case SUPER_MOVE:
                // m_State.setAnimation(a_MedFlashMagenta);
                m_State.setColor(Color.kMagenta);
                m_stateColor = Color.kMagenta.toHexString();
                break;

            case ALIGNING:
                // m_State.setAnimation(a_MedFlashCyan);
                m_State.setColor(Color.kCyan);
                m_stateColor = Color.kCyan.toHexString();
                break;

            case HAVE_CORAL:
                m_State.setColor(Color.kGreen);
                m_stateColor = Color.kGreen.toHexString();
                break;

            case ENABLED:
                // m_State.setAnimation(a_SingleFadeFastYellow);
                m_State.setColor(Color.kYellow);
                m_stateColor = Color.kYellow.toHexString();
                break;

            default:
                break;
        }
        m_currentState = newState;
    }

    /** Update Match Timer */
    @Override
    public void setMatchTimerState(MatchTimerState mtState)
    {
        // Match Timer Module
        // * Autonomous (15 sec): Flashing Yellow
        // * 2:15 -> 1:00: Solid Green
        // * 1:00 -> 0:20: Solid Yellow
        // * 0:20 -> 0:10: Solid Red
        // * 0:10 -> 0:00: Strobing Red
        // * Non-auto periods & Disabled: White

        if (mtState != m_mtState) {

            switch (mtState) {

                case OFF:
                    m_MatchTime.setOff();
                    break;
                case TELEOP_BEGIN:
                    m_MatchTime.setColor(Color.kGreen);
                    break;
                case LAST_60:
                    m_MatchTime.setColor(Color.kYellow);
                    break;
                case LAST_20:
                    m_MatchTime.setColor(Color.kRed);
                    break;
                case LAST_10:
                    m_MatchTime.setAnimation(a_TimeExpiring);
                    break;
                case END:
                default:
                    m_MatchTime.setColor(Color.kWhite);
                    break;
            }
            m_mtState = mtState;
        }
    }

    // Convert WPILIb Color components to CANdle input values
    private int getR(Color WPILibColor)
    {
        return (int) (WPILibColor.red * 255);
    }

    private int getG(Color WPILibColor)
    {
        return (int) (WPILibColor.green * 255);
    }

    private int getB(Color WPILibColor)
    {
        return (int) (WPILibColor.blue * 255);
    }

    /*
     * LED Segments
     */
    class LEDSegment {

        int startIndex;
        int segmentSize;
        int animationSlot;

        private LEDSegment(int startIndex, int segmentSize, int animationSlot)
        {
            this.startIndex = startIndex;
            this.segmentSize = segmentSize;
            this.animationSlot = animationSlot;
        }

        public void setColor(Color color)
        {
            m_candle.clearAnimation(animationSlot);
            m_candle.setLEDs(getR(color), getG(color), getB(color), 0, startIndex, segmentSize);
            m_candle.modulateVBatOutput(0.95);
        }

        private void setAnimation(Animation animation)
        {
            m_candle.clearAnimation(animationSlot);
            m_candle.animate(animation, animationSlot);
            m_candle.modulateVBatOutput(0.95);
        }

        public void setOff()
        {
            m_candle.clearAnimation(animationSlot);
            m_candle.setLEDs(0, 0, 0, 0, startIndex, segmentSize);
            m_candle.modulateVBatOutput(0.0);
        }
    }

    // Disabled Animations
    Animation a_RightRedLarson =
        new LarsonAnimation(getR(Color.kRed), getG(Color.kRed), getB(Color.kRed), 0, 0.2,
            m_FullRight.segmentSize, BounceMode.Front, 10, m_FullRight.startIndex);
    Animation a_LeftRedLarson =
        new LarsonAnimation(getR(Color.kRed), getG(Color.kRed), getB(Color.kRed), 0, 0.2,
            m_FullLeft.segmentSize, BounceMode.Front, 10, m_FullLeft.startIndex);
    Animation a_RightBlueLarson =
        new LarsonAnimation(getR(Color.kBlue), getG(Color.kBlue), getB(Color.kBlue), 0, 0.2,
            m_FullRight.segmentSize, BounceMode.Front, 10, m_FullRight.startIndex);
    Animation a_LeftBlueLarson =
        new LarsonAnimation(getR(Color.kBlue), getG(Color.kBlue), getB(Color.kBlue), 0,
            0.2, m_FullLeft.segmentSize, BounceMode.Front, 10, m_FullLeft.startIndex);
    Animation a_RightRainbow =
        new RainbowAnimation(0.7, 0.5, m_FullRight.segmentSize, true, m_FullRight.startIndex);
    Animation a_LeftRainbow =
        new RainbowAnimation(0.7, 0.5, m_FullLeft.segmentSize, false, m_FullLeft.startIndex);
    Animation a_RightFlame =
        new FireAnimation(1.0, 0.75, m_FullRight.segmentSize, 1.0, 0.1, true,
            m_FullRight.startIndex);
    Animation a_LeftFlame =
        new FireAnimation(1.0, 0.75, m_FullLeft.segmentSize, 1.0, 0.1, false,
            m_FullLeft.startIndex);

    // Robot State Animations
    // Processor
    Animation a_FastFlashGreen =
        new StrobeAnimation(getR(Color.kGreen), getG(Color.kGreen), getB(Color.kGreen),
            0, 0.8, m_State.segmentSize, m_State.startIndex);
    // Intaking
    Animation a_FastFlashRed =
        new StrobeAnimation(getR(Color.kRed), getG(Color.kRed), getB(Color.kRed),
            0, 0.8, m_State.segmentSize, m_State.startIndex);
    // Climbing
    Animation a_SlowFlashRed =
        new StrobeAnimation(getR(Color.kRed), getG(Color.kRed), getB(Color.kRed),
            0, 0.2, m_State.segmentSize, m_State.startIndex);
    // Super Move
    Animation a_MedFlashMagenta =
        new StrobeAnimation(getR(Color.kMagenta), getG(Color.kMagenta), getB(Color.kMagenta),
            0, 0.5, m_State.segmentSize, m_State.startIndex);
    // Aligning
    Animation a_MedFlashCyan =
        new StrobeAnimation(getR(Color.kCyan), getG(Color.kCyan), getB(Color.kCyan),
            0, 0.5, m_State.segmentSize, m_State.startIndex);
    // Enabled
    Animation a_SingleFadeFastYellow =
        new SingleFadeAnimation(getR(Color.kYellow), getG(Color.kYellow), getB(Color.kYellow),
            0, 0.8, m_State.segmentSize, m_State.startIndex);

    // Match Timer Animations
    Animation a_InAutonomous =
        new StrobeAnimation(getR(Color.kYellow), getG(Color.kYellow), getB(Color.kYellow),
            0, 0.8, m_MatchTime.segmentSize, m_MatchTime.startIndex);
    Animation a_TimeExpiring =
        new StrobeAnimation(getR(Color.kRed), getG(Color.kRed), getB(Color.kRed),
            0, 0.5, m_MatchTime.segmentSize, m_MatchTime.startIndex);

}
