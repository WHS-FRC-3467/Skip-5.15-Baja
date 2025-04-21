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

    AddressableLED led;
    AddressableLEDBuffer ledBuffer;

    LEDState currentState = LEDState.NOT_SET;
    GPMode currentGPMode = GPMode.CORAL;
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

    // Define LED Buffer Viewa
    // Numbering Sequence: Right LEDs go down, Left LEDs go Up
    // Match Time Indicator - CANdle module
    AddressableLEDBufferView MatchTime;
    // These are for Disabled/Auto states
    // They are each a full strip
    AddressableLEDBufferView FullRight;
    AddressableLEDBufferView FullLeft;
    // These are for Coral/Algae Mode while robot is Enabled
    // These are the top pixels on each strip
    AddressableLEDBufferView RightTip;
    AddressableLEDBufferView LeftTip;
    // This is for what the robot is doing while robot is Enabled
    // This is the bottom of each strip combined into one segment
    AddressableLEDBufferView State;

    // Create LED patterns that sets the entire strip to a solid color
    LEDPattern solidBlack = LEDPattern.solid(Color.kBlack);
    LEDPattern solidWhite = LEDPattern.solid(Color.kWhite);
    LEDPattern solidRed = LEDPattern.solid(Color.kRed);
    LEDPattern solidBlue = LEDPattern.solid(Color.kBlue);
    LEDPattern solidGreen = LEDPattern.solid(Color.kGreen);
    LEDPattern solidYellow = LEDPattern.solid(Color.kYellow);
    LEDPattern solidCyan = LEDPattern.solid(Color.kCyan);
    LEDPattern solidMagenta = LEDPattern.solid(Color.kMagenta);
    LEDPattern solidOrange = LEDPattern.solid(Color.kOrange);
    LEDPattern solidFirstRed = LEDPattern.solid(Color.kFirstRed);
    LEDPattern solidFirstBlue = LEDPattern.solid(Color.kFirstBlue);
    LEDPattern solidAlliance = LEDPattern.solid(Color.kFirstRed);
    LEDPattern fastFlashGreen = LEDPattern.solid(Color.kGreen).blink(Seconds.of(0.3));
    LEDPattern slowFlashRed = LEDPattern.solid(Color.kRed).blink(Seconds.of(0.7));
    LEDPattern fastFlashRed = LEDPattern.solid(Color.kRed).blink(Seconds.of(0.3));
    LEDPattern medFlashYellow = LEDPattern.solid(Color.kYellow).blink(Seconds.of(0.5));

    // Create Rainbow Pattern - all hues at maximum saturation and half brightness
    private final LEDPattern rainbow = LEDPattern.rainbow(255, 128);
    // Rainbow animation
    // Our LED strip has a density of 144 LEDs per meter
    private final Distance kLedSpacing = Meters.of(1 / 144.0);
    // Create a new pattern that scrolls the rainbow pattern across the LED strip, moving at a
    // speed of 0.5 meter per second.
    private final LEDPattern scrollingRainbow =
        rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(.5), kLedSpacing);

    public LEDSubsystemIOWPILib()
    {
        // Must be a PWM header, not MXP or DIO
        led = new AddressableLED(9);

        // Reusable buffer
        // Length is expensive to set, so only set it once, then just update data
        ledBuffer = new AddressableLEDBuffer(TOTAL_LEDS);
        led.setLength(ledBuffer.getLength());

        // Set the data
        led.setData(ledBuffer);
        led.start();

        // Create Buffer Views
        MatchTime = ledBuffer.createView(MATCHTIME_START, MATCHTIME_END);
        // These are for Disabled/Auto states
        // They are each a full strip
        FullRight = ledBuffer.createView(RIGHTSTRIP_START, RIGHTSTRIP_END).reversed();
        FullLeft = ledBuffer.createView(LEFTSTRIP_START, LEFTSTRIP_END);
        // These are for Coral/Algae Mode while robot is Enabled
        // These are the top pixels on each strip
        RightTip = ledBuffer.createView(RIGHTTIP_START, RIGHTTIP_END).reversed();
        LeftTip = ledBuffer.createView(LEFTTIP_START, LEFTTIP_END);
        // This is for what the robot is doing while robot is Enabled
        // This is the bottom of each strip combined into one segment
        State = ledBuffer.createView(STATE_START, STATE_END);
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
        solidAlliance = LEDPattern.solid(allianceColor);

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
                newGPMode = GPMode.NOT_SET;
                tipColor = Color.kBlack.toHexString();
                break;
            default:
                if (newGPMode != currentGPMode) {
                    switch (newGPMode) {
                        case ALGAE:
                            solidGreen.applyTo(LeftTip);
                            solidGreen.applyTo(RightTip);
                            tipColor = Color.kGreen.toHexString();
                            break;
                        case CORAL:
                        default:
                            solidWhite.applyTo(LeftTip);
                            solidWhite.applyTo(RightTip);
                            tipColor = Color.kWhite.toHexString();
                            break;
                    }
                    currentGPMode = newGPMode;
                    led.setData(ledBuffer);
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
        if (currentState == newState) {
            return;
        }

        // Process and make changes for changed LEDState
        switch (newState) {
            case DISABLED:
                scrollingRainbow.applyTo(FullLeft);
                scrollingRainbow.applyTo(FullRight);
                tipColor = allianceColor.toHexString();
                stateColor = allianceColor.toHexString();
                break;

            case DISABLED_FAR:
                fastFlashRed.applyTo(FullLeft);
                stateColor = Color.kRed.toHexString();
                break;

            case DISABLED_TRANSLATION_OK:
                solidGreen.applyTo(FullLeft);
                solidAlliance.applyTo(FullRight);
                tipColor = Color.kGreen.toHexString();
                stateColor = allianceColor.toHexString();
                break;

            case DISABLED_ROTATION_OK:
                solidAlliance.applyTo(FullLeft);
                solidGreen.applyTo(FullRight);
                tipColor = allianceColor.toHexString();
                stateColor = Color.kGreen.toHexString();
                break;

            case DISABLED_BOTH_OK:
                solidGreen.applyTo(FullLeft);
                solidGreen.applyTo(FullRight);
                tipColor = Color.kGreen.toHexString();
                stateColor = Color.kGreen.toHexString();
                break;

            case AUTONOMOUS:
                medFlashYellow.applyTo(FullLeft);
                medFlashYellow.applyTo(FullRight);
                solidYellow.applyTo(MatchTime);
                tipColor = Color.kOrange.toHexString();
                stateColor = Color.kOrange.toHexString();
                break;

            case INTAKING:
                slowFlashRed.applyTo(State);
                stateColor = Color.kRed.toHexString();
                break;

            case CLIMBING:
                fastFlashRed.applyTo(State);
                stateColor = Color.kRed.toHexString();
                break;

            case CLIMBED:
                solidGreen.applyTo(State);
                stateColor = Color.kGreen.toHexString();
                break;

            case SUPER_MOVE:
                solidMagenta.applyTo(State);
                stateColor = Color.kMagenta.toHexString();
                break;

            case ALIGNING:
                solidCyan.applyTo(State);
                stateColor = Color.kCyan.toHexString();
                break;

            case HAVE_CORAL:
                solidGreen.applyTo(State);
                stateColor = Color.kGreen.toHexString();
                break;

            case ENABLED:
                solidYellow.applyTo(State);
                stateColor = Color.kYellow.toHexString();
                break;

            default:
                break;
        }
        currentState = newState;
        led.setData(ledBuffer);
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
                    solidBlack.applyTo(MatchTime);
                    break;
                case TELEOP_BEGIN:
                    solidGreen.applyTo(MatchTime);
                    break;
                case LAST_60:
                    solidYellow.applyTo(MatchTime);
                    break;
                case LAST_20:
                    solidRed.applyTo(MatchTime);
                    break;
                case LAST_10:
                    fastFlashRed.applyTo(MatchTime);
                    break;
                case END:
                default:
                    solidWhite.applyTo(MatchTime);
                    break;
            }
            this.mtState = mtState;
            led.setData(ledBuffer);
        }
    }
}
