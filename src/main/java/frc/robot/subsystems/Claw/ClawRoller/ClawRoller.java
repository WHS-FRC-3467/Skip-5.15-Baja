package frc.robot.subsystems.Claw.ClawRoller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem.TargetState;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class ClawRoller
    extends GenericMotionProfiledSubsystem<ClawRoller.State> {

    private static final LoggedTunableNumber L1_SPEED =
        new LoggedTunableNumber("ClawRoller/L1 Speed", 0.6);

    public final Trigger stalled =
        new Trigger(
            () -> (Math.abs(super.inputs.velocityRps) <= 0.02
                && super.inputs.supplyCurrentAmps[0] >= 1));

    public final Trigger stopped =
        new Trigger(() -> (Math.abs(super.inputs.velocityRps) <= 0.02));

    public final Trigger freeSpin =
        new Trigger(() -> (Math.abs(super.inputs.velocityRps) >= 10));

    @RequiredArgsConstructor
    @Getter
    public enum State implements TargetState {
        OFF(new ProfileType.DISABLED_BRAKE()),
        INTAKE(new ProfileType.OPEN_CURRENT(() -> 80,
            () -> .35)),
        SHUFFLE(new ProfileType.POSITION(() -> -0.3, 0)),
        L4_RETRACT(new ProfileType.POSITION(() -> -0.5, 0)),
        SCORE(new ProfileType.OPEN_VOLTAGE(() -> 2.0)),
        L1_SHUFFLE(new ProfileType.POSITION(() -> 0.5, 0)),
        L1_SCORE(new ProfileType.OPEN_CURRENT(() -> 60, L1_SPEED)),
        HOLDCORAL(new ProfileType.POSITION(() -> -0.1, 0)),
        ALGAE_FORWARD(new ProfileType.OPEN_CURRENT(() -> 100, () -> 1.0)),
        ALGAE_REVERSE(new ProfileType.OPEN_CURRENT(() -> -90, () -> 1.0));

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
        return runOnce(() -> this.state = state).withName("ClawRoller Set State: " + state.name());
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

    public Command L4ShuffleCommand()
    {
        return Commands.sequence(
            Commands.runOnce(() -> this.io.zeroSensors()),
            this.setStateCommand(State.L4_RETRACT));
    }

    public Command L1ShuffleCommand()
    {
        return Commands.sequence(
            Commands.runOnce(() -> this.io.zeroSensors()),
            this.setStateCommand(State.L1_SHUFFLE));
    }
}
