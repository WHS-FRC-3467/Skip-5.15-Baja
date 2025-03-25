package frc.robot.subsystems.Arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem.TargetState;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class Arm extends GenericMotionProfiledSubsystem<Arm.State> {

    @RequiredArgsConstructor
    @Getter
    public enum State implements TargetState {
        STOW(new ProfileType.MM_POSITION(() -> Units.degreesToRotations(125.18), 0)),
        CORAL_INTAKE(new ProfileType.MM_POSITION(() -> Units.degreesToRotations(135.7), 0)),
        LEVEL_1(new ProfileType.MM_POSITION(() -> Units.degreesToRotations(94.13), 0)),
        LEVEL_2(new ProfileType.MM_POSITION(() -> Units.degreesToRotations(94.48), 0)),
        LEVEL_3(new ProfileType.MM_POSITION(() -> Units.degreesToRotations(104.48), 0)),
        LEVEL_4(new ProfileType.MM_POSITION(() -> Units.degreesToRotations(101.33), 0)),
        CLIMB(new ProfileType.MM_POSITION(() -> Units.degreesToRotations(82.4), 0)),
        ALGAE_LOW(new ProfileType.MM_POSITION(() -> Units.degreesToRotations(103.3), 0)),
        ALGAE_LOW_P(new ProfileType.MM_POSITION(() -> .2377, 0)),
        ALGAE_HIGH(new ProfileType.MM_POSITION(() -> Units.degreesToRotations(103.3), 0)),
        ALGAE_HIGH_P(new ProfileType.MM_POSITION(() -> .2446, 0)),
        ALGAE_GROUND(new ProfileType.MM_POSITION(() -> Units.degreesToRotations(70.0), 0)),
        PROCESSOR_SCORE(
            new ProfileType.MM_POSITION(() -> Units.degreesToRotations(120.0), 0)),
        BARGE(new ProfileType.MM_POSITION(() -> Units.degreesToRotations(130.0), 0)),
        COAST(new ProfileType.DISABLED_COAST()),
        BRAKE(new ProfileType.DISABLED_BRAKE());

        private final ProfileType profileType;
    }

    @Getter
    @Setter
    private State state = State.STOW;

    public Arm(ArmIO io, boolean isSim)
    {
        super(State.STOW.profileType, ArmConstants.kSubSysConstants, io, isSim);
        SmartDashboard.putData("Arm Coast Command", setCoastStateCommand());
        SmartDashboard.putData("Arm Brake Command", setBrakeStateCommand());
    }

    /** Constructor */
    public Command setStateCommand(State state)
    {
        return this.runOnce(() -> this.state = state);
    }

    public Command setCoastStateCommand()
    {
        return this.runOnce(() -> this.state = State.COAST);
    }

    public Command setBrakeStateCommand()
    {
        return this.runOnce(() -> this.state = State.BRAKE);
    }

    public boolean atPosition(double tolerance)
    {
        return io.atPosition(state.profileType, tolerance);
    }

}
