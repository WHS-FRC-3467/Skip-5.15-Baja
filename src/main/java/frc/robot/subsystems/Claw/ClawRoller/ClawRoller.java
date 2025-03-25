package frc.robot.subsystems.Claw.ClawRoller;

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
public class ClawRoller
    extends GenericMotionProfiledSubsystem<ClawRoller.State> {

    public final Trigger stalled =
        new Trigger(
            () -> (Math.abs(super.inputs.velocityRps) <= 0.02
                && super.inputs.supplyCurrentAmps[0] >= 1));

    public final Trigger stopped =
        new Trigger(() -> (Math.abs(super.inputs.velocityRps) <= 0.02));

    @RequiredArgsConstructor
    @Getter
    public enum State implements TargetState {
        OFF(new ProfileType.DISABLED_BRAKE()),
        INTAKE(new ProfileType.OPEN_CURRENT(() -> 80,
            () -> .35)),
        SHUFFLE(new ProfileType.POSITION(() -> -0.05, 0)),
        SCORE(new ProfileType.OPEN_VOLTAGE(() -> 4.0)),
        HOLDCORAL(new ProfileType.POSITION(() -> -0.05, 0)),
        ALGAE_FORWARD(new ProfileType.OPEN_CURRENT(() -> 90, () -> 0.6)),
        ALGAE_REVERSE(new ProfileType.OPEN_CURRENT(() -> -90, () -> 0.6));

        private final ProfileType profileType;
    }

    @Getter
    private State state = State.OFF;

    public ClawRoller(ClawRollerIO io, boolean isSim)
    {
        super(State.OFF.profileType, ClawRollerConstants.kSubSysConstants, io, isSim);
    }

    public Command setStateCommand(State state)
    {
        return runOnce(() -> this.state = state);
    }

    public boolean atPosition(double tolerance)
    {
        return io.atPosition(state.profileType, tolerance);
    }

    public Command shuffleCommand()
    {
        return Commands.sequence(
            Commands.runOnce(() -> this.io.zeroSensors()),
            this.setStateCommand(State.SHUFFLE));
    }
}
