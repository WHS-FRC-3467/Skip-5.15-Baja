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
        // HOME is climber upright, Prep - Assuming that PREP position is parallel to the x axis,
        // CLIMB is inwards
        HOME(new ProfileType.MM_POSITION(() -> 0, 0)),
        PREP(new ProfileType.MM_POSITION(() -> -200, 0)),
        CLIMB(new ProfileType.MM_POSITION(() -> -15, 0)),
        ClIMB_MORE(new ProfileType.MM_POSITION(() -> -5, 0)),
        HOMING(new ProfileType.OPEN_VOLTAGE(() -> 4));

        private final ProfileType profileType;
    }

    @Getter
    @Setter
    private State state = State.HOME;

    public Climber(ClimberIO io, boolean isSim)
    {
        super(State.HOME.profileType, ClimberConstants.kSubSysConstants, io, isSim);
        SmartDashboard.putData("Home Climber Command", getHomeCommand());
    }

    public Command setStateCommand(State state)
    {
        return this.runOnce(() -> this.state = state);
    }

    // Climbing Triggers
    public boolean climbRequested = false; // Whether or not a climb request is active
    private Trigger climbRequest = new Trigger(() -> climbRequested); // Trigger for climb request
    public int climbStep = 0; // Tracking what step in the climb sequence we are on, is at zero when
                              // not climbing

    // Triggers for each step of the climb sequence
    private Trigger climbStep1 = new Trigger(() -> climbStep == 1);
    private Trigger climbStep2 = new Trigger(() -> climbStep == 2);
    private Trigger climbStep3 = new Trigger(() -> climbStep == 3);

    // Debouncer and trigger checks to see if the climber has finished climbing
    private Debouncer climbedDebouncer = new Debouncer(.25, DebounceType.kRising);

    public Trigger climbedTrigger =
        new Trigger(
            () -> climbedDebouncer.calculate(
                this.state == State.CLIMB
                    && (Math.abs(io.getSupplyCurrent()) > ClimberConstants.kSupplyCurrentLimit)));

    public boolean atPosition(double tolerance)
    {
        return io.atPosition(state.profileType, tolerance);
    }

    private Debouncer homedDebouncer = new Debouncer(0.1, DebounceType.kRising);
    private Debouncer stateDebouncer = new Debouncer(1, DebounceType.kRising);

    private Trigger homedTrigger =
        new Trigger(() -> homedDebouncer
            .calculate(Math.abs(super.inputs.torqueCurrentAmps[0]) > 10)
            && stateDebouncer.calculate(this.state == State.HOMING));

    private Command getHomeCommand()
    {
        return this.setStateCommand(State.HOMING)
            .andThen(Commands.waitUntil(homedTrigger))
            .andThen(Commands.runOnce(() -> io.zeroSensors()))
            .andThen(this.setStateCommand(State.HOME));
    }
}
