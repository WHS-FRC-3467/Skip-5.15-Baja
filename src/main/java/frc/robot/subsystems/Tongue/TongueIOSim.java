package frc.robot.subsystems.Tongue;

import frc.robot.subsystems.GenericMotorSubsystem.GenericMotorSubsystemIOImpl;

public class TongueIOSim extends GenericMotorSubsystemIOImpl implements TongueIO {

    public TongueIOSim()
    {
        super(TongueConstants.kSubSysConstants, true);
    }
}
