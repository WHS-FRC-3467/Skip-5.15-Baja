package frc.robot.subsystems.Arm;

import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemIOImpl;

public class ArmIOTalonFX extends GenericMotionProfiledSubsystemIOImpl implements ArmIO {

    public ArmIOTalonFX()
    {
        super(ArmConstants.kSubSysConstants, false);
    }
}
