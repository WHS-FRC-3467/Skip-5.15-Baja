package frc.robot.subsystems.Arm;

import frc.robot.subsystems.GenericMotorSubsystem.GenericMotorSubsystemIOImpl;

public class ArmIOSim extends GenericMotorSubsystemIOImpl implements ArmIO {

    public ArmIOSim()
    {
        super(ArmConstants.kSubSysConstants, true);
    }
}
