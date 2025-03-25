package frc.robot.subsystems.Climber;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem.TargetState;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class Climber extends GenericMotionProfiledSubsystem<Climber.State> {

    @RequiredArgsConstructor
    @Getter
    public enum State implements TargetState {
        HOLD(new ProfileType.DISABLED_BRAKE()),
        HOME(new ProfileType.MM_POSITION(() -> 0, 0)),
        PREP(new ProfileType.POSITION(() -> 200, 0)),
        CLIMB(new ProfileType.MM_POSITION(() -> 15, 0)),
        MANUAL_CLIMB(new ProfileType.OPEN_VOLTAGE(() -> 12)),
        HOMING(new ProfileType.OPEN_VOLTAGE(() -> 4));

        private final ProfileType profileType;
    }

    @Getter
    private State state = State.HOME;

    public Climber(ClimberIO io, boolean isSim)
    {
        super(State.HOME.profileType, ClimberConstants.kSubSysConstants, io, isSim);
        SmartDashboard.putData("Climber Home Command", homeCommand());
        SmartDashboard.putData("Climber Zero Command", zeroPositionCommand());
    }

    public Command setStateCommand(State state)
    {
        return this.runOnce(() -> this.state = state);
    }

    public boolean atPosition(double tolerance)
    {
        return io.atPosition(state.profileType, tolerance);
    }

    public Command zeroPositionCommand()
    {
        return Commands.runOnce(() -> io.zeroSensors());
    }

    public Command setClimbRequestCommand(boolean enabled)
    {
        return Commands.runOnce(() -> this.climbRequested = enabled);
    }

    public Command indexClimbState()
    {
        return Commands.runOnce(() -> this.climbStep += 1);
    }

    public Command resetClimb()
    {
        return Commands.runOnce(() -> {
            this.climbRequested = false;
            this.climbStep = 0;
        }).andThen(this.setStateCommand(State.HOME));
    }

    // Climbing Triggers
    private boolean climbRequested = false; // Whether or not a climb request is active
    private Trigger climbRequest = new Trigger(() -> climbRequested); // Trigger for climb request
    private int climbStep = 0; // Tracking what step in the climb sequence we are on, is at zero
                               // when
                               // not climbing

    // Triggers for each step of the climb sequence
    public Trigger climbStep1 = new Trigger(() -> climbStep == 1);
    public Trigger climbStep2 = new Trigger(() -> climbStep == 2);
    public Trigger climbStep3 = new Trigger(() -> climbStep == 3);

    private Debouncer homedDebouncer = new Debouncer(0.1, DebounceType.kRising);
    private Debouncer stateDebouncer = new Debouncer(1, DebounceType.kRising);

    private Trigger homedTrigger =
        new Trigger(() -> homedDebouncer
            .calculate(Math.abs(super.inputs.torqueCurrentAmps[0]) > 10)
            && stateDebouncer.calculate(this.state == State.HOMING));

    private Command homeCommand()
    {
        return Commands.sequence(
            this.setStateCommand(State.HOMING),
            Commands.waitUntil(homedTrigger),
            this.zeroPositionCommand(),
            this.setStateCommand(State.HOME));
    }
}
