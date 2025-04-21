package frc.robot.subsystems.Climber;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.GenericMotorSubsystem.GenericMotorSubsystem;
import frc.robot.subsystems.GenericMotorSubsystem.GenericMotorSubsystem.TargetState;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class Climber extends GenericMotorSubsystem<Climber.State> {

    private static final LoggedTunableNumber climbHeight =
        new LoggedTunableNumber("Climber/ClimbHeight", -0.1);

    @RequiredArgsConstructor
    @Getter
    public enum State implements TargetState {
        HOLD(new ProfileType.DISABLED_BRAKE()),
        HOME(new ProfileType.POSITION(() -> 0, 0)),
        PREP(new ProfileType.POSITION(() -> -1.4, 0)),
        CLIMB(new ProfileType.MM_POSITION(climbHeight, 1)),
        MANUAL_CLIMB(new ProfileType.OPEN_VOLTAGE(() -> 8)),
        HOMING(new ProfileType.OPEN_VOLTAGE(() -> 4));

        private final ProfileType profileType;
    }

    @Getter
    private State state = State.HOME;

    public Climber(ClimberIO io, boolean isSim)
    {
        super(State.HOME.profileType, ClimberConstants.kSubSysConstants, io, isSim);
        SmartDashboard.putData("Climber Home Command", homeCommand());
        SmartDashboard.putData("Climber Zero Command",
            zeroPositionCommand().ignoringDisable(true));
    }

    public Command setStateCommand(State state)
    {
        return this.runOnce(() -> this.state = state);
    }

    public Command climbMore()
    {
        return this.startEnd(() -> setState(State.MANUAL_CLIMB), () -> setState(State.HOLD));
    }

    public boolean atPosition(double tolerance)
    {
        return io.atPosition(state.profileType, tolerance);
    }

    public Command zeroPositionCommand()
    {
        return Commands
            .sequence(Commands.runOnce(() -> io.zeroSensors()), this.setStateCommand(State.HOME));
    }

    private Debouncer homedDebouncer = new Debouncer(0.75, DebounceType.kRising);

    private Trigger homedTrigger =
        new Trigger(() -> homedDebouncer
            .calculate(Math.abs(super.inputs.supplyCurrentAmps[0]) >= 4.7));

    private Command homeCommand()
    {
        return Commands.sequence(
            this.setStateCommand(State.HOMING),
            Commands.waitUntil(homedTrigger))
            .finallyDo(() -> {
                io.zeroSensors();
                this.setState(State.HOME);
            });
    }
}
