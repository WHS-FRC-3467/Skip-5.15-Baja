package frc.robot.subsystems.Claw.ClawRoller;

import frc.robot.subsystems.GenericMotorSubsystem.GenericMotorSubsystemIOImpl;

public class ClawRollerIOSim extends GenericMotorSubsystemIOImpl
    implements ClawRollerIO {

    public ClawRollerIOSim()
    {
        super(ClawRollerConstants.kSubSysConstants, true);
    }
}
