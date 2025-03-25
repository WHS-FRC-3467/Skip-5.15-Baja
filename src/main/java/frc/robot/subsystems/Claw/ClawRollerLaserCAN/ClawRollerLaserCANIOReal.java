package frc.robot.subsystems.Claw.ClawRollerLaserCAN;

import frc.robot.subsystems.GenericLaserCANSubsystem.GenericLaserCANSubsystemIOImpl;

public class ClawRollerLaserCANIOReal extends GenericLaserCANSubsystemIOImpl
    implements ClawRollerLaserCANIO {

    public ClawRollerLaserCANIOReal()
    {
        super(ClawRollerLaserCANConstants.kSubSysConstants, false);
    }
}
