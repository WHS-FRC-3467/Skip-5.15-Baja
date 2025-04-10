package frc.robot.subsystems.Hopper.Hopper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem.TargetState;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class Hopper
    extends GenericMotionProfiledSubsystem<Hopper.State> {

    private static final LoggedTunableNumber speed =
        new LoggedTunableNumber("Hopper/Speed", 8.0);

    @RequiredArgsConstructor
    @Getter
    public enum State implements TargetState {
        OFF(new ProfileType.DISABLED_BRAKE()),
        INTAKE(new ProfileType.OPEN_VOLTAGE(speed));

        private final ProfileType profileType;
    }

    @Getter
    private State state = State.OFF;

    public Hopper(HopperIO io, boolean isSim)
    {
        super(State.OFF.profileType, HopperConstants.kSubSysConstants, io, isSim);
    }

    public Command setStateCommand(State state)
    {
        return runOnce(() -> this.state = state).withName("Hopper Set State: " + state.name());
    }

    public boolean atPosition(double tolerance)
    {
        return io.atPosition(state.profileType, tolerance);
    }
}
