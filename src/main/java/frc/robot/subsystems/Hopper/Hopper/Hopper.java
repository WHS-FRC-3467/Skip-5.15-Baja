package frc.robot.subsystems.Hopper.Hopper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem.TargetState;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class Hopper
    extends GenericMotionProfiledSubsystem<Hopper.State> {

    @RequiredArgsConstructor
    @Getter
    public enum State implements TargetState {
        OFF(new ProfileType.DISABLED_BRAKE()),
        INTAKE(new ProfileType.OPEN_VOLTAGE(() -> 12));

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
