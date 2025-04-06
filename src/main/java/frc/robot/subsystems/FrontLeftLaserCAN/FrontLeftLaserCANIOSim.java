package frc.robot.subsystems.FrontLeftLaserCAN;

import frc.robot.subsystems.GenericLaserCANSubsystem.GenericLaserCANSubsystemIOImpl;

public class FrontLeftLaserCANIOSim extends GenericLaserCANSubsystemIOImpl
    implements FrontLeftLaserCANIO {

    public FrontLeftLaserCANIOSim()
    {
        super(FrontLeftLaserCANConstants.kSubSysConstants, true);
    }
}
