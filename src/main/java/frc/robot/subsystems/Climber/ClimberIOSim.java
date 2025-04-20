package frc.robot.subsystems.Climber;

import frc.robot.subsystems.GenericMotorSubsystem.GenericMotorSubsystemIOImpl;

public class ClimberIOSim extends GenericMotorSubsystemIOImpl implements ClimberIO {

    public ClimberIOSim()
    {
        super(ClimberConstants.kSubSysConstants, true);
    }
}
