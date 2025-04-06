package frc.robot.subsystems.FrontLeftLaserCAN;

import frc.robot.subsystems.GenericLaserCANSubsystem.GenericLaserCANSubsystemIOImpl;

public class FrontLeftLaserCANIOReal extends GenericLaserCANSubsystemIOImpl
    implements FrontLeftLaserCANIO {

    public FrontLeftLaserCANIOReal()
    {
        super(FrontLeftLaserCANConstants.kSubSysConstants, false);
    }
}
