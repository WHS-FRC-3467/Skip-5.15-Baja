package frc.robot.subsystems.Tongue;

import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemIOImpl;

public class TongueIOSim extends GenericMotionProfiledSubsystemIOImpl implements TongueIO {

    public TongueIOSim()
    {
        super(TongueConstants.kSubSysConstants, true);
    }
}
