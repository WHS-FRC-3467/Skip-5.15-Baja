package frc.robot.subsystems.Hopper.Hopper;

import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemIOImpl;

public class HopperIOSim extends GenericMotionProfiledSubsystemIOImpl
    implements HopperIO {

    public HopperIOSim()
    {
        super(HopperConstants.kSubSysConstants, true);
    }
}
