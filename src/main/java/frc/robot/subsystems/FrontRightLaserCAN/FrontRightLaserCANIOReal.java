package frc.robot.subsystems.FrontRightLaserCAN;

import frc.robot.subsystems.GenericLaserCANSubsystem.GenericLaserCANSubsystemIOImpl;

public class FrontRightLaserCANIOReal extends GenericLaserCANSubsystemIOImpl
    implements FrontRightLaserCANIO {

    public FrontRightLaserCANIOReal()
    {
        super(FrontRightLaserCANConstants.kSubSysConstants, false);
    }
}
