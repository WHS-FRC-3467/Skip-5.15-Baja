package frc.robot.subsystems.Arm;

import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemIOImpl;

public class ArmIOSim extends GenericMotionProfiledSubsystemIOImpl implements ArmIO {

    public ArmIOSim()
    {
        super(ArmConstants.kSubSysConstants, true);
    }
}
