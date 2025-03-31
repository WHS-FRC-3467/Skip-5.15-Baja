package frc.robot.subsystems.Tongue;

import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemIOImpl;

public class TongueIOTalonFX extends GenericMotionProfiledSubsystemIOImpl implements TongueIO {

    public TongueIOTalonFX()
    {
        super(TongueConstants.kSubSysConstants, false);
    }
}
