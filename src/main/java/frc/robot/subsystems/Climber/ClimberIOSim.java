package frc.robot.subsystems.Climber;

import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemIOImpl;

public class ClimberIOSim extends GenericMotionProfiledSubsystemIOImpl implements ClimberIO {

    public ClimberIOSim()
    {
        super(ClimberConstants.kSubSysConstants, true);
    }
}
