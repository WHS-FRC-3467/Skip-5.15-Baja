package frc.robot.subsystems.Claw.ClawRoller;

import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemIOImpl;

public class ClawRollerIOTalonFX extends GenericMotionProfiledSubsystemIOImpl
    implements ClawRollerIO {

    public ClawRollerIOTalonFX()
    {
        super(ClawRollerConstants.kSubSysConstants, false);
    }
}
