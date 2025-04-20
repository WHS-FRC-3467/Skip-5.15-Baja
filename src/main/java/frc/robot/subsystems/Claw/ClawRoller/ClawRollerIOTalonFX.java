package frc.robot.subsystems.Claw.ClawRoller;

import frc.robot.subsystems.GenericMotorSubsystem.GenericMotorSubsystemIOImpl;

public class ClawRollerIOTalonFX extends GenericMotorSubsystemIOImpl
    implements ClawRollerIO {

    public ClawRollerIOTalonFX()
    {
        super(ClawRollerConstants.kSubSysConstants, false);
    }
}
