package frc.robot.subsystems.LED;

import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Claw.ClawRoller.ClawRoller;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.LED.LEDSubsystemIO.AllianceColor;
import frc.robot.subsystems.LED.LEDSubsystemIO.GPMode;
import frc.robot.subsystems.LED.LEDSubsystemIO.LEDState;
import frc.robot.subsystems.LED.LEDSubsystemIO.MatchTimerState;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.util.LoggedTunableNumber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveCommands.DriveMode;

public class LEDSubsystem extends SubsystemBase {

    // Subsystems to query
    ClawRoller m_ClawRoller;
    Arm m_Arm;
    Elevator m_Elevator;
    Climber m_Climber;
    Vision m_Vision;

    Trigger m_haveCoral;
    Trigger m_isCoralMode;
    Trigger m_isProcessorMode;

    // LoggedTunableNumbers for testing LED states
    private LoggedTunableNumber kMode, kState;
    // Flag for testing mode
    boolean kTesting = false;

    AllianceColor m_DSAlliance = AllianceColor.UNDETERMINED;
    LEDSubsystemIO m_io;

    protected final LEDSubsystemIOInputsAutoLogged inputs =
        new LEDSubsystemIOInputsAutoLogged();

    /*
     * Constructor Creates a new LEDSubsystem
     */
    public LEDSubsystem(
        LEDSubsystemIO io,
        ClawRoller clawRoller,
        Arm arm,
        Elevator elevator,
        Climber climber,
        Vision vision,
        Trigger haveCoral,
        Trigger isCoralMode,
        Trigger isProcessorMode)
    {

        m_io = io;
        m_ClawRoller = clawRoller;
        m_Arm = arm;
        m_Elevator = elevator;
        m_Climber = climber;
        m_Vision = vision;
        m_haveCoral = haveCoral;
        m_isCoralMode = isCoralMode;
        m_isProcessorMode = isProcessorMode;

        // Tunable numbers for testing
        kMode = new LoggedTunableNumber("LED/Mode", 0);
        kState = new LoggedTunableNumber("LED/State", 0);
    }

    @Override
    public void periodic()
    {
        LEDState newState;
        GPMode newGPMode;

        // Determine and Set Alliance color
        if (m_DSAlliance == AllianceColor.UNDETERMINED) {
            if (DriverStation.getAlliance().isPresent()) {
                if (DriverStation.getAlliance().get() == Alliance.Blue) {
                    m_DSAlliance = AllianceColor.BLUE;
                } else {
                    m_DSAlliance = AllianceColor.RED;
                }
            }
            m_io.setAlliance(m_DSAlliance);
        }

        if (kTesting) {
            // Testing Mode - change values using Tunable Numbers
            newState = testLEDState((int) kState.get());
            switch ((int) kMode.get()) {
                case 1:
                    newGPMode = GPMode.ALGAE;
                    break;
                case 2:
                    newGPMode = GPMode.PROCESSOR;
                    break;
                default:
                    newGPMode = GPMode.CORAL;
                    break;
            }

            if (newState == LEDState.ENABLED) {
                runMatchTimerPattern();
            } else if (newState == LEDState.DISABLED) {
                this.timerDisabled();
            }

        } else {

            // Real Robot
            // Determine Game Piece Mode
            if (m_isCoralMode.getAsBoolean()) {
                newGPMode = GPMode.CORAL;
            } else if (m_isProcessorMode.getAsBoolean()) {
                newGPMode = GPMode.PROCESSOR;
            } else {
                newGPMode = GPMode.ALGAE;
            }
            // Get latest robot state
            newState = getRobotState();
        }

        // Process Changes
        m_io.setGPMode(newGPMode);
        m_io.setRobotState(newState);

        // Do AKit logging
        m_io.updateInputs(inputs);
        Logger.processInputs("LED", inputs);
    }

    // Determine the current state of the robot
    private LEDState getRobotState()
    {
        // --- Order of Priorities ---
        // Whole Robot:
        // - DISABLED_BOTH_OK
        // - DISABLED_TRANSLATION_OK
        // - DISABLED_ROTATION_OK
        // - DISABLED
        // - AUTONOMOUS
        // Operating State:
        // - INTAKING
        // - FEEDING
        // - CLIMBING
        // - CLIMBED
        // - SUPER_MOVE
        // - ALIGNING
        // - HAVE_CORAL
        // - ENABLED
        // Game Piece Mode:
        // - GPMode

        LEDState newState = LEDState.DISABLED;

        // Determine state of robot to be displayed
        if (DriverStation.isDisabled()) {

            // Use LEDs while Disabled to indicate proper lineup
            boolean goodTrans = SmartDashboard.getBoolean("Alignment/Translation", false);
            boolean goodRot = SmartDashboard.getBoolean("Alignment/Rotation", false);
            if (goodTrans && goodRot) {
                newState = LEDState.DISABLED_BOTH_OK;
            } else if (goodTrans) {
                newState = LEDState.DISABLED_TRANSLATION_OK;
            } else if (goodRot) {
                newState = LEDState.DISABLED_ROTATION_OK;
            } else {
                newState = LEDState.DISABLED;
            }
            this.timerDisabled();

        } else if (DriverStation.isAutonomousEnabled()) {
            // In Autonomous mode
            newState = LEDState.AUTONOMOUS;
            this.timerDisabled();

        } else {
            // If not Disabled or in Auto, determine robot state

            // Run MatchTimer
            runMatchTimerPattern();

            // Intaking Coral?
            if (m_ClawRoller.getState() == ClawRoller.State.INTAKE) {
                if (!m_haveCoral.getAsBoolean()) {
                    // Waiting for Coral
                    newState = LEDState.INTAKING;
                }

                // Climbing?
            } else if (m_Climber.getState() == Climber.State.PREP ||
                m_Climber.getState() == Climber.State.CLIMB) {
                // Climb complete?
                if (m_Climber.atPosition(0.1)) {
                    newState = LEDState.CLIMBED;
                } else {
                    newState = LEDState.CLIMBING;
                }

                // Moving Superstructure?
            } else if (m_Elevator.isElevated()) {
                if (!m_Elevator.atPosition(0.0) || !m_Arm.atPosition(0.0)) {
                    // An Elevated position has been commanded, but it's not there yet
                    newState = LEDState.SUPER_MOVE;
                }

                // Aligning?
            } else if (DriveCommands.getDriveMode() == DriveMode.dmApproach) {
                // The robot is auto-aligning
                newState = LEDState.ALIGNING;

                // Holding Coral?
            } else if (m_haveCoral.getAsBoolean()) {
                // Claw is holding Coral
                newState = LEDState.HAVE_CORAL;

            } else {
                // Default state: Just Enabled
                newState = LEDState.ENABLED;
            }
        }
        return newState;
    }


    // Match Timer Module
    Timer m_pseudoTimer = new Timer();
    MatchTimerState m_currentState = MatchTimerState.OFF;

    private void runMatchTimerPattern()
    {
        MatchTimerState newState = MatchTimerState.END;

        double matchTime = DriverStation.getMatchTime();
        if (matchTime < 0.0) {
            m_pseudoTimer.start();
            matchTime = (int) (150.0 - m_pseudoTimer.get());
        }

        if (matchTime > 60.0) {
            // * 2:15 -> 1:00: Beginning of Teleop
            newState = MatchTimerState.TELEOP_BEGIN;
        } else if (matchTime > 20.0) {
            // * 1:00 -> 0:20: One minute to go
            newState = MatchTimerState.LAST_60;
        } else if (matchTime > 10.0) {
            // * 0:20 -> 0:10: 20 seconds to go
            newState = MatchTimerState.LAST_20;
        } else if (matchTime > 0.0) {
            // * 0:10 -> 0:00: 10 seconds to go
            newState = MatchTimerState.LAST_10;
        } else {
            // * Match has ended
            newState = MatchTimerState.END;
        }

        if (newState != m_currentState) {
            m_io.setMatchTimerState(newState);
            m_currentState = newState;
        }
    }

    public void timerDisabled()
    {
        m_pseudoTimer.stop();
        m_pseudoTimer.reset();
        if (m_currentState != MatchTimerState.OFF) {
            m_io.setMatchTimerState(MatchTimerState.OFF);
            m_currentState = MatchTimerState.OFF;
        }
    }

    LEDState testLEDState(int stateNum)
    {
        switch (stateNum) {
            default:
            case 0:
                return LEDState.DISABLED;
            case 1:
                return LEDState.DISABLED_TRANSLATION_OK;
            case 2:
                return LEDState.DISABLED_ROTATION_OK;
            case 3:
                return LEDState.DISABLED_BOTH_OK;
            case 4:
                return LEDState.AUTONOMOUS;
            case 5:
                return LEDState.INTAKING;
            case 6:
                return LEDState.CLIMBING;
            case 7:
                return LEDState.CLIMBED;
            case 8:
                return LEDState.SUPER_MOVE;
            case 9:
                return LEDState.ALIGNING;
            case 10:
                return LEDState.HAVE_CORAL;
            case 11:
                return LEDState.ENABLED;
        }
    }

}

