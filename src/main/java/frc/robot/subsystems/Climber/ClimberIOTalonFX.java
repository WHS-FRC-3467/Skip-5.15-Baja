package frc.robot.subsystems.Climber;

import frc.robot.subsystems.GenericMotorSubsystem.GenericMotorSubsystemIOImpl;

public class ClimberIOTalonFX extends GenericMotorSubsystemIOImpl implements ClimberIO {

    public ClimberIOTalonFX()
    {
        super(ClimberConstants.kSubSysConstants, false);
    }
}
