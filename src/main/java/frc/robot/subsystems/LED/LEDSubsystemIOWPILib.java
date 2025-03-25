// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LED;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class LEDSubsystemIOWPILib implements LEDSubsystemIO {

    AddressableLED m_led;
    AddressableLEDBuffer m_ledBuffer;

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

    // Define LED Buffer Viewa
    // Numbering Sequence: Right LEDs go down, Left LEDs go Up
    // Match Time Indicator - CANdle module
    AddressableLEDBufferView m_MatchTime;
    // These are for Disabled/Auto states
    // They are each a full strip
    AddressableLEDBufferView m_FullRight;
    AddressableLEDBufferView m_FullLeft;
    // These are for Coral/Algae Mode while robot is Enabled
    // These are the top pixels on each strip
    AddressableLEDBufferView m_RightTip;
    AddressableLEDBufferView m_LeftTip;
    // This is for what the robot is doing while robot is Enabled
    // This is the bottom of each strip combined into one segment
    AddressableLEDBufferView m_State;

    // Create LED patterns that sets the entire strip to a solid color
    LEDPattern m_solidBlack = LEDPattern.solid(Color.kBlack);
    LEDPattern m_solidWhite = LEDPattern.solid(Color.kWhite);
    LEDPattern m_solidRed = LEDPattern.solid(Color.kRed);
    LEDPattern m_solidBlue = LEDPattern.solid(Color.kBlue);
    LEDPattern m_solidGreen = LEDPattern.solid(Color.kGreen);
    LEDPattern m_solidYellow = LEDPattern.solid(Color.kYellow);
    LEDPattern m_solidCyan = LEDPattern.solid(Color.kCyan);
    LEDPattern m_solidMagenta = LEDPattern.solid(Color.kMagenta);
    LEDPattern m_solidOrange = LEDPattern.solid(Color.kOrange);
    LEDPattern m_solidFirstRed = LEDPattern.solid(Color.kFirstRed);
    LEDPattern m_solidFirstBlue = LEDPattern.solid(Color.kFirstBlue);
    LEDPattern m_solidAlliance = LEDPattern.solid(Color.kFirstRed);
    LEDPattern m_fastFlashGreen = LEDPattern.solid(Color.kGreen).blink(Seconds.of(0.3));
    LEDPattern m_slowFlashRed = LEDPattern.solid(Color.kRed).blink(Seconds.of(0.7));
    LEDPattern m_fastFlashRed = LEDPattern.solid(Color.kRed).blink(Seconds.of(0.3));
    LEDPattern m_medFlashYellow = LEDPattern.solid(Color.kYellow).blink(Seconds.of(0.5));

    // Create Rainbow Pattern - all hues at maximum saturation and half brightness
    private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
    // Rainbow animation
    // Our LED strip has a density of 144 LEDs per meter
    private final Distance kLedSpacing = Meters.of(1 / 144.0);
    // Create a new pattern that scrolls the rainbow pattern across the LED strip, moving at a
    // speed of 0.5 meter per second.
    private final LEDPattern m_scrollingRainbow =
        m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(.5), kLedSpacing);

    public LEDSubsystemIOWPILib()
    {
        // Must be a PWM header, not MXP or DIO
        m_led = new AddressableLED(9);

        // Reusable buffer
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(TOTAL_LEDS);
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();

        // Create Buffer Views
        m_MatchTime = m_ledBuffer.createView(MATCHTIME_START, MATCHTIME_END);
        // These are for Disabled/Auto states
        // They are each a full strip
        m_FullRight = m_ledBuffer.createView(RIGHTSTRIP_START, RIGHTSTRIP_END).reversed();
        m_FullLeft = m_ledBuffer.createView(LEFTSTRIP_START, LEFTSTRIP_END);
        // These are for Coral/Algae Mode while robot is Enabled
        // These are the top pixels on each strip
        m_RightTip = m_ledBuffer.createView(RIGHTTIP_START, RIGHTTIP_END).reversed();
        m_LeftTip = m_ledBuffer.createView(LEFTTIP_START, LEFTTIP_END);
        // This is for what the robot is doing while robot is Enabled
        // This is the bottom of each strip combined into one segment
        m_State = m_ledBuffer.createView(STATE_START, STATE_END);
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
        m_solidAlliance = LEDPattern.solid(m_allianceColor);

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
                            m_fastFlashGreen.applyTo(m_LeftTip);
                            m_fastFlashGreen.applyTo(m_RightTip);
                            m_tipColor = Color.kAqua.toHexString();
                            break;
                        case ALGAE:
                            m_solidGreen.applyTo(m_LeftTip);
                            m_solidGreen.applyTo(m_RightTip);
                            m_tipColor = Color.kGreen.toHexString();
                            break;
                        case CORAL:
                        default:
                            m_solidWhite.applyTo(m_LeftTip);
                            m_solidWhite.applyTo(m_RightTip);
                            m_tipColor = Color.kWhite.toHexString();
                            break;
                    }
                    m_currentGPMode = newGPMode;
                    m_led.setData(m_ledBuffer);
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
        // - SUPER_MOVE -> Magenta Flash Medium
        // - ALIGNING -> Cyan Flash Medium
        // - HAVE_CORAL -> Green
        // - ENABLED -> Yellow

        // Don't do anything unless state has changed
        if (m_currentState == newState) {
            return;
        }

        // Process and make changes for changed LEDState
        switch (newState) {
            case DISABLED:
                m_scrollingRainbow.applyTo(m_FullLeft);
                m_scrollingRainbow.applyTo(m_FullRight);
                m_tipColor = m_allianceColor.toHexString();
                m_stateColor = m_allianceColor.toHexString();
                break;

            case DISABLED_TRANSLATION_OK:
                m_solidGreen.applyTo(m_FullLeft);
                m_solidAlliance.applyTo(m_FullRight);
                m_tipColor = Color.kGreen.toHexString();
                m_stateColor = m_allianceColor.toHexString();
                break;

            case DISABLED_ROTATION_OK:
                m_solidAlliance.applyTo(m_FullLeft);
                m_solidGreen.applyTo(m_FullRight);
                m_tipColor = m_allianceColor.toHexString();
                m_stateColor = Color.kGreen.toHexString();
                break;

            case DISABLED_BOTH_OK:
                m_solidGreen.applyTo(m_FullLeft);
                m_solidGreen.applyTo(m_FullRight);
                m_tipColor = Color.kGreen.toHexString();
                m_stateColor = Color.kGreen.toHexString();
                break;

            case AUTONOMOUS:
                m_medFlashYellow.applyTo(m_FullLeft);
                m_medFlashYellow.applyTo(m_FullRight);
                m_solidYellow.applyTo(m_MatchTime);
                m_tipColor = Color.kOrange.toHexString();
                m_stateColor = Color.kOrange.toHexString();
                break;

            case INTAKING:
                m_slowFlashRed.applyTo(m_State);
                m_stateColor = Color.kRed.toHexString();
                break;

            case CLIMBING:
                m_fastFlashRed.applyTo(m_State);
                m_stateColor = Color.kRed.toHexString();
                break;

            case CLIMBED:
                m_solidGreen.applyTo(m_State);
                m_stateColor = Color.kGreen.toHexString();
                break;

            case SUPER_MOVE:
                m_solidMagenta.applyTo(m_State);
                m_stateColor = Color.kMagenta.toHexString();
                break;

            case ALIGNING:
                m_solidCyan.applyTo(m_State);
                m_stateColor = Color.kCyan.toHexString();
                break;

            case HAVE_CORAL:
                m_solidGreen.applyTo(m_State);
                m_stateColor = Color.kGreen.toHexString();
                break;

            case ENABLED:
                m_solidYellow.applyTo(m_State);
                m_stateColor = Color.kYellow.toHexString();
                break;

            default:
                break;
        }
        m_currentState = newState;
        m_led.setData(m_ledBuffer);
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
                    m_solidBlack.applyTo(m_MatchTime);
                    break;
                case TELEOP_BEGIN:
                    m_solidGreen.applyTo(m_MatchTime);
                    break;
                case LAST_60:
                    m_solidYellow.applyTo(m_MatchTime);
                    break;
                case LAST_20:
                    m_solidRed.applyTo(m_MatchTime);
                    break;
                case LAST_10:
                    m_fastFlashRed.applyTo(m_MatchTime);
                    break;
                case END:
                default:
                    m_solidWhite.applyTo(m_MatchTime);
                    break;
            }
            m_mtState = mtState;
            m_led.setData(m_ledBuffer);
        }
    }
}
