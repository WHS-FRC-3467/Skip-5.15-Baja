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

public class LEDSubsystem extends SubsystemBase {

    // Subsystems to query
    ClawRoller clawRoller;
    Arm arm;
    Elevator elevator;
    Climber climber;
    Vision vision;

    Trigger haveCoral;
    Trigger isCoralMode;

    // LoggedTunableNumbers for testing LED states
    private LoggedTunableNumber kMode, kState;
    // Flag for testing mode
    boolean kTesting = false;

    AllianceColor DSAlliance = AllianceColor.UNDETERMINED;
    LEDSubsystemIO io;

    int visionOutCounter = 0;
    Timer timer = new Timer();
    double lastTimeStamp = 0.0;
    double thisTimeStamp;

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
        Trigger isCoralMode)
    {

        this.io = io;
        this.clawRoller = clawRoller;
        this.arm = arm;
        this.elevator = elevator;
        this.climber = climber;
        this.vision = vision;
        this.haveCoral = haveCoral;
        this.isCoralMode = isCoralMode;

        // Tunable numbers for testing
        kMode = new LoggedTunableNumber("LED/Mode", 0);
        kState = new LoggedTunableNumber("LED/State", 0);
        timer.start();
    }

    @Override
    public void periodic()
    {
        // Only loop through periodic LED checking every 0.2 seconds, makes code more efficient
        thisTimeStamp = timer.get();
        if (thisTimeStamp - lastTimeStamp >= 0.2) {
            lastTimeStamp = thisTimeStamp;
            LEDState newState;
            GPMode newGPMode;

            // Determine and Set Alliance color
            if (DSAlliance == AllianceColor.UNDETERMINED) {
                if (DriverStation.getAlliance().isPresent()) {
                    if (DriverStation.getAlliance().get() == Alliance.Blue) {
                        DSAlliance = AllianceColor.BLUE;
                    } else {
                        DSAlliance = AllianceColor.RED;
                    }
                }
                io.setAlliance(DSAlliance);
            }

            if (kTesting) {
                // Testing Mode - change values using Tunable Numbers
                newState = testLEDState((int) kState.get());
                switch ((int) kMode.get()) {
                    case 1:
                        newGPMode = GPMode.ALGAE;
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
                if (isCoralMode.getAsBoolean()) {
                    newGPMode = GPMode.CORAL;
                } else {
                    newGPMode = GPMode.ALGAE;
                }
                // Get latest robot state
                newState = getRobotState();
            }

            // Process Changes
            io.setGPMode(newGPMode);
            io.setRobotState(newState);

            // Do AKit logging
            io.updateInputs(inputs);
            Logger.processInputs("LED", inputs);
            Logger.recordOutput("LED/GamePiece", inputs.GamePiece);
            Logger.recordOutput("LED/RobotState", inputs.RobotState);
        }
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
        // - VISION_OUT
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
                // } else if (SmartDashboard.getNumber("Alignment/Distance To Auto Start", 0) >=
                // Units
                // .metersToInches(1)) {
                // newState = LEDState.DISABLED_FAR;
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

            // Vision Out? For 2 seconds, quickly flash the LEDs red
            if (vision.hasVision.negate().getAsBoolean()) {
                if (visionOutCounter < 10) {
                    visionOutCounter++;
                    newState = LEDState.VISION_OUT;
                    // End the function so newState doesn't get overwritten
                    return newState;
                }
            } else {
                // Vision is back, reset flashing counter
                visionOutCounter = 0;
            }
            // Intaking Coral?
            if (clawRoller.getState() == ClawRoller.State.INTAKE &&
                !haveCoral.getAsBoolean()) {
                // Waiting for Coral
                newState = LEDState.INTAKING;

                // Climbing?
            } else if (climber.getState() == Climber.State.PREP ||
                climber.getState() == Climber.State.CLIMB) {
                // Climb complete?
                if (climber.atPosition(0.1)) {
                    newState = LEDState.CLIMBED;
                } else {
                    newState = LEDState.CLIMBING;
                }

                // Moving Superstructure?
            } else if (elevator.isElevated() &&
                (!elevator.atPosition(0.0) ||
                    !arm.atPosition(0.0))) {
                // An Elevated position has been commanded, but it's not there yet
                newState = LEDState.SUPER_MOVE;

                // Holding Coral?
            } else if (haveCoral.getAsBoolean()) {
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
    Timer pseudoTimer = new Timer();
    MatchTimerState currentState = MatchTimerState.OFF;

    private void runMatchTimerPattern()
    {
        MatchTimerState newState = MatchTimerState.END;

        double matchTime = DriverStation.getMatchTime();
        if (matchTime < 0.0) {
            pseudoTimer.start();
            matchTime = (int) (150.0 - pseudoTimer.get());
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

        if (newState != currentState) {
            io.setMatchTimerState(newState);
            currentState = newState;
        }
    }

    public void timerDisabled()
    {
        pseudoTimer.stop();
        pseudoTimer.reset();
        if (currentState != MatchTimerState.OFF) {
            io.setMatchTimerState(MatchTimerState.OFF);
            currentState = MatchTimerState.OFF;
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
                return LEDState.VISION_OUT;
            case 6:
                return LEDState.INTAKING;
            case 7:
                return LEDState.CLIMBING;
            case 8:
                return LEDState.CLIMBED;
            case 9:
                return LEDState.SUPER_MOVE;
            case 10:
                return LEDState.ALIGNING;
            case 11:
                return LEDState.HAVE_CORAL;
            case 12:
                return LEDState.ENABLED;
        }
    }

}

