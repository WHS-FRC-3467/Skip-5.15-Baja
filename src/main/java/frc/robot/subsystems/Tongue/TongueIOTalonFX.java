package frc.robot.subsystems.Tongue;

import frc.robot.subsystems.GenericMotorSubsystem.GenericMotorSubsystemIOImpl;

public class TongueIOTalonFX extends GenericMotorSubsystemIOImpl implements TongueIO {

    public TongueIOTalonFX()
    {
        super(TongueConstants.kSubSysConstants, false);
    }
}
