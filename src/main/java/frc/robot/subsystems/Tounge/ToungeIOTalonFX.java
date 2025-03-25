package frc.robot.subsystems.Tounge;

import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemIOImpl;

public class ToungeIOTalonFX extends GenericMotionProfiledSubsystemIOImpl implements ToungeIO {

    public ToungeIOTalonFX()
    {
        super(ToungeConstants.kSubSysConstants, false);
    }
}
