package frc.robot.subsystems.Tounge;

import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemIOImpl;

public class ToungeIOSim extends GenericMotionProfiledSubsystemIOImpl implements ToungeIO {

    public ToungeIOSim()
    {
        super(ToungeConstants.kSubSysConstants, true);
    }
}
