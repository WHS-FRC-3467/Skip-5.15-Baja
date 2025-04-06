package frc.robot.subsystems.FrontRightLaserCAN;

import frc.robot.subsystems.GenericLaserCANSubsystem.GenericLaserCANSubsystemIOImpl;

public class FrontRightLaserCANIOSim extends GenericLaserCANSubsystemIOImpl
    implements FrontRightLaserCANIO {

    public FrontRightLaserCANIOSim()
    {
        super(FrontRightLaserCANConstants.kSubSysConstants, true);
    }
}
