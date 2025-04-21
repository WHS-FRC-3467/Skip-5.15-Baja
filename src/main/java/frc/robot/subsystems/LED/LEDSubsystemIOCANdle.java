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
    private static final CANdle candle = new CANdle(Ports.ELEVATOR_CANDLE.getDeviceNumber());

    Alert ledConfigError = new Alert("CANdle Configuration Error!", Alert.AlertType.kWarning);

    boolean teleopBegun = false;
    LEDState currentState = LEDState.NOT_SET;
    GPMode currentGPMode = GPMode.NOT_SET;
    AllianceColor DSAlliance = AllianceColor.UNDETERMINED;
    Color allianceColor = Color.kBlack;
    MatchTimerState mtState = MatchTimerState.END;
    String tipColor = "x000000";
    String stateColor = "x000000";

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
    LEDSegment MatchTime = new LEDSegment(MATCHTIME_START, BASESEG_SIZE, 0);
    // These are for Disabled/Auto states
    // They are each a full strip
    LEDSegment FullRight = new LEDSegment(RIGHTSTRIP_START, FULLSTRIP_SIZE, 1);
    LEDSegment FullLeft = new LEDSegment(LEFTSTRIP_START, FULLSTRIP_SIZE, 2);
    // These are for Coral/Algae Mode while robot is Enabled
    // These are the top pixels on each strip
    LEDSegment RightTip = new LEDSegment(RIGHTTIP_START, TIP_SIZE, 3);
    LEDSegment LeftTip = new LEDSegment(LEFTTIP_START, TIP_SIZE, 4);
    // This is for what the robot is doing while robot is Enabled
    // This is the bottom of each strip combined into one segment
    LEDSegment State = new LEDSegment(STATE_START, (STATE_END - STATE_START) + 1, 5);

    public LEDSubsystemIOCANdle()
    {
        candle.configFactoryDefault();

        CANdleConfiguration candleConfiguration = new CANdleConfiguration();
        // candle.getAllConfigs(candleConfiguration);

        candleConfiguration.statusLedOffWhenActive = true;
        candleConfiguration.disableWhenLOS = false;
        candleConfiguration.stripType = LEDStripType.RGB;
        candleConfiguration.brightnessScalar = 0.5;
        candleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;
        candleConfiguration.v5Enabled = false;
        ErrorCode ec = candle.configAllSettings(candleConfiguration, 100);
        if (ec != ErrorCode.OK) {
            ledConfigError.set(true);
            ledConfigError.setText(ec.toString());
        }

        // StripType setting needs to be done twice (for some reason, once doesn't work)
        ec = candle.configLEDType(LEDStripType.RGB, 300);
        if (ec != ErrorCode.OK) {
            ledConfigError.set(true);
            ledConfigError.setText(ec.toString());
        }
    }

    @Override
    public void updateInputs(LEDSubsystemIOInputs inputs)
    {
        inputs.alliance = DSAlliance;
        inputs.ledState = currentState;
        inputs.gpMode = currentGPMode;
        inputs.matchTime = mtState;
        inputs.GamePiece = tipColor;
        inputs.RobotState = stateColor;
    }

    /** Update Alliance Color */
    @Override
    public void setAlliance(AllianceColor alliance)
    {
        switch (alliance) {
            case RED:
                allianceColor = Color.kRed;
                break;
            case BLUE:
                allianceColor = Color.kBlue;
                break;
            case UNDETERMINED:
            default:
                allianceColor = Color.kMagenta;
                break;
        }
        DSAlliance = alliance;
    }

    // Set the Game Piece Mode indicator LEDS
    @Override
    public void setGPMode(GPMode newGPMode)
    {
        // Mode:
        // - GPMode -> Tips: White or "algae" color

        // Process and make changes for changed GPMode
        switch (currentState) {
            case DISABLED:
            case DISABLED_FAR:
            case DISABLED_TRANSLATION_OK:
            case DISABLED_ROTATION_OK:
            case DISABLED_BOTH_OK:
            case AUTONOMOUS:
                // Mode is not displayed in these cases
                // so just break out
                currentGPMode = GPMode.NOT_SET;
                tipColor = Color.kBlack.toHexString();
                break;
            default:
                if (newGPMode != currentGPMode) {

                    if (!teleopBegun) {
                        FullLeft.setOff();
                        FullRight.setOff();
                        teleopBegun = true;
                    }
                    switch (newGPMode) {
                        case ALGAE:
                            LeftTip.setColor(Color.kGreen);
                            RightTip.setColor(Color.kGreen);
                            tipColor = Color.kGreen.toHexString();
                            break;
                        case CORAL:
                        default:
                            LeftTip.setColor(Color.kWhite);
                            RightTip.setColor(Color.kWhite);
                            tipColor = Color.kWhite.toHexString();
                            break;
                    }
                    currentGPMode = newGPMode;
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
        // - VISION_OUT -> Orange Flash
        // - INTAKING -> Red Flash Slow
        // - FEEDING -> Blue
        // - CLIMBING -> Red Flash Fast
        // - CLIMBED -> Green
        // - SUPER_MOVE -> Magenta
        // - ALIGNING -> Cyan
        // - HAVE_CORAL -> Green
        // - ENABLED -> Yellow

        // Don't do anything unless state has changed
        if (currentState == newState) {
            return;
        }

        // Process and make changes for changed LEDState
        switch (newState) {
            case DISABLED:
                FullLeft.setColor(allianceColor);
                FullRight.setColor(allianceColor);
                tipColor = allianceColor.toHexString();
                stateColor = allianceColor.toHexString();
                break;

            case DISABLED_FAR:
                FullLeft.setAnimation(a_StrobeRedLeft);
                FullRight.setAnimation(a_StrobeRedRight);
                break;

            case DISABLED_TRANSLATION_OK:
                FullLeft.setColor(Color.kGreen);
                FullRight.setColor(allianceColor);
                tipColor = Color.kGreen.toHexString();
                stateColor = allianceColor.toHexString();
                break;

            case DISABLED_ROTATION_OK:
                FullLeft.setColor(allianceColor);
                FullRight.setColor(Color.kGreen);
                tipColor = allianceColor.toHexString();
                stateColor = Color.kGreen.toHexString();
                break;

            case DISABLED_BOTH_OK:
                FullLeft.setColor(Color.kGreen);
                FullRight.setColor(Color.kGreen);
                tipColor = Color.kGreen.toHexString();
                stateColor = Color.kGreen.toHexString();
                break;

            case AUTONOMOUS:
                FullLeft.setAnimation(a_LeftFlame);
                FullRight.setAnimation(a_RightFlame);
                MatchTime.setAnimation(a_InAutonomous);
                tipColor = Color.kOrange.toHexString();
                stateColor = Color.kOrange.toHexString();
                break;

            case VISION_OUT:
                State.setAnimation(a_FastFlashOrange);
                stateColor = Color.kOrange.toHexString();
                break;

            case INTAKING:
                State.setAnimation(a_SlowFlashRed);
                stateColor = Color.kRed.toHexString();
                break;

            case CLIMBING:
                State.setAnimation(a_FastFlashRed);
                stateColor = Color.kRed.toHexString();
                break;

            case CLIMBED:
                State.setColor(Color.kGreen);
                stateColor = Color.kGreen.toHexString();
                break;

            case SUPER_MOVE:
                // State.setAnimation(a_MedFlashMagenta);
                State.setColor(Color.kMagenta);
                stateColor = Color.kMagenta.toHexString();
                break;

            case ALIGNING:
                // State.setAnimation(a_MedFlashCyan);
                State.setColor(Color.kCyan);
                stateColor = Color.kCyan.toHexString();
                break;

            case HAVE_CORAL:
                State.setColor(Color.kGreen);
                stateColor = Color.kGreen.toHexString();
                break;

            case ENABLED:
                // State.setAnimation(a_SingleFadeFastYellow);
                State.setColor(Color.kYellow);
                stateColor = Color.kYellow.toHexString();
                break;

            default:
                break;
        }
        currentState = newState;
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

        if (this.mtState != mtState) {

            switch (mtState) {

                case OFF:
                    MatchTime.setOff();
                    break;
                case TELEOP_BEGIN:
                    MatchTime.setColor(Color.kGreen);
                    break;
                case LAST_60:
                    MatchTime.setColor(Color.kYellow);
                    break;
                case LAST_20:
                    MatchTime.setColor(Color.kRed);
                    break;
                case LAST_10:
                    MatchTime.setAnimation(a_TimeExpiring);
                    break;
                case END:
                default:
                    MatchTime.setColor(Color.kWhite);
                    break;
            }
            this.mtState = mtState;
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
            candle.clearAnimation(animationSlot);
            candle.setLEDs(getR(color), getG(color), getB(color), 0, startIndex, segmentSize);
            candle.modulateVBatOutput(0.95);
        }

        private void setAnimation(Animation animation)
        {
            candle.clearAnimation(animationSlot);
            candle.animate(animation, animationSlot);
            candle.modulateVBatOutput(0.95);
        }

        public void setOff()
        {
            candle.clearAnimation(animationSlot);
            candle.setLEDs(0, 0, 0, 0, startIndex, segmentSize);
            candle.modulateVBatOutput(0.0);
        }
    }

    // Disabled Animations
    Animation a_StrobeRedLeft =
        new StrobeAnimation(getR(Color.kRed), getG(Color.kRed), getB(Color.kRed),
            0, 0.2, FullLeft.segmentSize, FullLeft.startIndex);
    Animation a_StrobeRedRight =
        new StrobeAnimation(getR(Color.kRed), getG(Color.kRed), getB(Color.kRed),
            0, 0.2, FullRight.segmentSize, FullRight.startIndex);
    Animation a_RightRedLarson =
        new LarsonAnimation(getR(Color.kRed), getG(Color.kRed), getB(Color.kRed), 0, 0.2,
            FullRight.segmentSize, BounceMode.Front, 10, FullRight.startIndex);
    Animation a_LeftRedLarson =
        new LarsonAnimation(getR(Color.kRed), getG(Color.kRed), getB(Color.kRed), 0, 0.2,
            FullLeft.segmentSize, BounceMode.Front, 10, FullLeft.startIndex);
    Animation a_RightBlueLarson =
        new LarsonAnimation(getR(Color.kBlue), getG(Color.kBlue), getB(Color.kBlue), 0, 0.2,
            FullRight.segmentSize, BounceMode.Front, 10, FullRight.startIndex);
    Animation a_LeftBlueLarson =
        new LarsonAnimation(getR(Color.kBlue), getG(Color.kBlue), getB(Color.kBlue), 0,
            0.2, FullLeft.segmentSize, BounceMode.Front, 10, FullLeft.startIndex);
    Animation a_RightRainbow =
        new RainbowAnimation(0.7, 0.5, FullRight.segmentSize, true, FullRight.startIndex);
    Animation a_LeftRainbow =
        new RainbowAnimation(0.7, 0.5, FullLeft.segmentSize, false, FullLeft.startIndex);
    Animation a_RightFlame =
        new FireAnimation(1.0, 0.75, FullRight.segmentSize, 1.0, 0.1, true,
            FullRight.startIndex);
    Animation a_LeftFlame =
        new FireAnimation(1.0, 0.75, FullLeft.segmentSize, 1.0, 0.1, false,
            FullLeft.startIndex);

    // GamePieceMode Animations
    // Processor
    Animation a_LeftFlashGreen =
        new StrobeAnimation(getR(Color.kGreen), getG(Color.kGreen), getB(Color.kGreen),
            0, 0.8, LeftTip.segmentSize, LeftTip.startIndex);
    Animation a_RightFlashGreen =
        new StrobeAnimation(getR(Color.kGreen), getG(Color.kGreen), getB(Color.kGreen),
            0, 0.8, RightTip.segmentSize, RightTip.startIndex);

    // Robot State Animations
    // Lost vision?
    Animation a_FastFlashOrange =
        new StrobeAnimation(getR(Color.kRed), getG(Color.kOrange), getB(Color.kOrange),
            0, 0.8, State.segmentSize, State.startIndex);
    // Intaking
    Animation a_FastFlashRed =
        new StrobeAnimation(getR(Color.kRed), getG(Color.kRed), getB(Color.kRed),
            0, 0.8, State.segmentSize, State.startIndex);
    // Climbing
    Animation a_SlowFlashRed =
        new StrobeAnimation(getR(Color.kRed), getG(Color.kRed), getB(Color.kRed),
            0, 0.2, State.segmentSize, State.startIndex);
    // Super Move
    Animation a_MedFlashMagenta =
        new StrobeAnimation(getR(Color.kMagenta), getG(Color.kMagenta), getB(Color.kMagenta),
            0, 0.5, State.segmentSize, State.startIndex);
    // Aligning
    Animation a_MedFlashCyan =
        new StrobeAnimation(getR(Color.kCyan), getG(Color.kCyan), getB(Color.kCyan),
            0, 0.5, State.segmentSize, State.startIndex);
    // Enabled
    Animation a_SingleFadeFastYellow =
        new SingleFadeAnimation(getR(Color.kYellow), getG(Color.kYellow), getB(Color.kYellow),
            0, 0.8, State.segmentSize, State.startIndex);

    // Match Timer Animations
    Animation a_InAutonomous =
        new StrobeAnimation(getR(Color.kYellow), getG(Color.kYellow), getB(Color.kYellow),
            0, 0.8, MatchTime.segmentSize, MatchTime.startIndex);
    Animation a_TimeExpiring =
        new StrobeAnimation(getR(Color.kRed), getG(Color.kRed), getB(Color.kRed),
            0, 0.5, MatchTime.segmentSize, MatchTime.startIndex);

}
