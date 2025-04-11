package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem.TargetState;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.sim.mechanisms.ArmElevComboReplay;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class Elevator extends GenericMotionProfiledSubsystem<Elevator.State> {

    ArmElevComboReplay m_Replay = null;

    static LoggedTunableNumber homingTuning =
        new LoggedTunableNumber("Elevator/HomingVoltageSP", -1);

    static LoggedTunableNumber launchHeight =
        new LoggedTunableNumber("Elevator/LaunchHeight", 3.4);

    @RequiredArgsConstructor
    @Getter
    public enum State implements TargetState {
        HOMING(new ProfileType.OPEN_VOLTAGE(() -> homingTuning.getAsDouble())),
        STOW(new ProfileType.MM_POSITION(() -> 0.09, 0)),
        CORAL_INTAKE(new ProfileType.MM_POSITION(() -> 0.0, 0)),
        LEVEL_1(new ProfileType.MM_POSITION(() -> 0.3, 0)),
        LEVEL_2(new ProfileType.MM_POSITION(() -> 1.217, 0)),
        LEVEL_3(new ProfileType.MM_POSITION(() -> 2.7, 0)),
        // LEVEL_4(new ProfileType.MM_POSITION(() -> 4.95, 0)), // UNH settings
        LEVEL_4(new ProfileType.MM_POSITION(() -> 5.11, 0)), // Toyota settings
        CLIMB(new ProfileType.MM_POSITION(() -> 0.0, 0)),
        ALGAE_LOW(new ProfileType.MM_POSITION(() -> 0.65, 0)),
        ALGAE_HIGH(new ProfileType.MM_POSITION(() -> 2.1, 0)),
        ALGAE_GROUND(new ProfileType.MM_POSITION(() -> 0.05, 0)),
        ALGAE_LOLLIPOP(new ProfileType.MM_POSITION(() -> 0.07, 0)),
        ALGAE_STOW(new ProfileType.MM_POSITION(() -> 0.2, 0)),
        PROCESSOR_SCORE(
            new ProfileType.MM_POSITION(() -> 0.05, 0)),
        BARGE(new ProfileType.MM_POSITION(() -> 5.6, 0)),
        CHARACTERIZATION(new ProfileType.CHARACTERIZATION()),
        COAST(new ProfileType.DISABLED_COAST()),
        BRAKE(new ProfileType.DISABLED_BRAKE());

        private final ProfileType profileType;
    }

    @Getter
    @Setter
    private State state = State.STOW;

    private RobotState robotState = RobotState.getInstance();

    public Elevator(ElevatorIO io, boolean isSim)
    {
        super(State.STOW.profileType, ElevatorConstants.kSubSysConstants, io, isSim);
        SmartDashboard.putData("Elevator Coast Command", setCoastStateCommand());
        SmartDashboard.putData("Elevator Brake Command", setBrakeStateCommand());

    }

    @Override
    public void periodic()
    {
        super.periodic();
        robotState.setElevatorHeight(this.io.getPosition());
    }

    public Command setStateCommand(State state)
    {
        return this.runOnce(() -> this.state = state)
            .withName("Elevator Set State: " + state.name());
    }

    public Command setCoastStateCommand()
    {
        return this.runOnce(() -> this.state = State.COAST);
    }

    public Command setBrakeStateCommand()
    {
        return this.runOnce(() -> this.state = State.BRAKE);
    }

    public boolean isElevated()
    {
        switch (this.getState()) {
            case LEVEL_1:
            case LEVEL_2:
            case LEVEL_3:
            case LEVEL_4:
            case CLIMB:
            case ALGAE_LOW:
            case ALGAE_HIGH:
            case ALGAE_GROUND:
            case PROCESSOR_SCORE:
            case BARGE:
                return true;

            default:
                return false;
        }
    }

    private Debouncer homedDebouncer = new Debouncer(0.1, DebounceType.kRising);

    public Trigger homedTrigger =
        new Trigger(
            () -> homedDebouncer.calculate(
                (this.state == State.HOMING && Math.abs(io.getSupplyCurrent()) > 3)));

    public Command getHomeCommand()
    {
        return this.setStateCommand(State.HOMING).andThen(Commands.waitUntil(homedTrigger))
            .andThen(this.zeroSensorCommand()).andThen(this.setStateCommand(State.STOW));
    }

    public Command zeroSensorCommand()
    {
        return new InstantCommand(() -> io.zeroSensors()).withName("Zero Elevator");
    }

    public boolean atPosition(double tolerance)
    {
        return io.atPosition(state.profileType, tolerance);
    }

    public Trigger launchHeightTrigger =
        new Trigger(() -> (io.getPosition() >= launchHeight.getAsDouble()));

}
