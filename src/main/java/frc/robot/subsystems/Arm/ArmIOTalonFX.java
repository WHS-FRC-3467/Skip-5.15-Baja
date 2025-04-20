package frc.robot.subsystems.Arm;

import frc.robot.subsystems.GenericMotorSubsystem.GenericMotorSubsystemIOImpl;

public class ArmIOTalonFX extends GenericMotorSubsystemIOImpl implements ArmIO {

    public ArmIOTalonFX()
    {
        super(ArmConstants.kSubSysConstants, false);
    }
}
