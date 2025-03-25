package frc.robot.subsystems.Tounge;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem.TargetState;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class Tounge extends GenericMotionProfiledSubsystem<Tounge.State> {

    static LoggedTunableNumber positionTuning =
        new LoggedTunableNumber("Tounge/PositionTuningSP", 10.0);

    static LoggedTunableNumber homingTuning =
        new LoggedTunableNumber("Tounge/HomingVoltageSP", -1);

    @RequiredArgsConstructor
    @Getter
    public enum State implements TargetState {
        HOMING(new ProfileType.OPEN_VOLTAGE(() -> homingTuning.getAsDouble())),
        STOW(new ProfileType.OPEN_VOLTAGE(() -> 0)),
        RAISED(new ProfileType.OPEN_VOLTAGE(() -> 1)),
        DOWN(new ProfileType.OPEN_VOLTAGE(() -> -12));

        private final ProfileType profileType;
    }

    @Getter
    @Setter
    private State state = State.STOW;

    public Tounge(ToungeIO io, boolean isSim)
    {
        super(State.STOW.profileType, ToungeConstants.kSubSysConstants, io, isSim);
        SmartDashboard.putData("Tounge homing command", homeCommand());
    }

    public Command setStateCommand(State state)
    {
        return this.runOnce(() -> this.state = state);
    }

    public boolean atPosition(double tolerance)
    {
        return io.atPosition(state.profileType, tolerance);
    }

    public Command zeroSensorCommand()
    {
        return new InstantCommand(() -> io.zeroSensors());
    }

    private Debouncer homedDebouncer = new Debouncer(0.1, DebounceType.kRising);

    public Trigger homedTrigger = new Trigger(
        () -> homedDebouncer.calculate(
            (this.state == State.HOMING && Math.abs(io.getSupplyCurrent()) > 2)));

    // public Trigger coralContactTrigger = new Trigger(
    // () -> atPosition(Units.degreesToRotations(5)) && Math.abs(io.getSupplyCurrent()) > 3);

    public Trigger coralContactTrigger = new Trigger(
        () -> MathUtil.isNear(.29, io.getPosition(), 0.1));

    public Trigger hasLoweredTrigger = new Trigger(
        () -> MathUtil.isNear(0, io.getPosition(), 0.1));

    public Command homeCommand()
    {
        return this.setStateCommand(State.HOMING)
            .andThen(Commands.waitUntil(homedTrigger).andThen(this.zeroSensorCommand())
                .andThen(this.setStateCommand(State.STOW)));
    }

    public Command lowerToungeCommand()
    {
        return Commands.sequence(
            this.setStateCommand(State.DOWN),
            Commands.race(
                Commands.waitUntil(this.hasLoweredTrigger),
                Commands.waitSeconds(0.5)),
            this.setStateCommand(State.STOW));
    }

}
