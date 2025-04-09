package frc.robot.subsystems.Hopper.Hopper;

import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemIOImpl;

public class HopperIOTalonFX extends GenericMotionProfiledSubsystemIOImpl
    implements HopperIO {

    public HopperIOTalonFX()
    {
        super(HopperConstants.kSubSysConstants, false);
    }
}
