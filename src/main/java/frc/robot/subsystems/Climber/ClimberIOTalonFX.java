package frc.robot.subsystems.Climber;

import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemIOImpl;

public class ClimberIOTalonFX extends GenericMotionProfiledSubsystemIOImpl implements ClimberIO {

    public ClimberIOTalonFX()
    {
        super(ClimberConstants.kSubSysConstants, false);
    }
}
