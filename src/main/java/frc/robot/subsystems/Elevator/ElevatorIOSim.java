package frc.robot.subsystems.Elevator;

import frc.robot.subsystems.GenericMotorSubsystem.GenericMotorSubsystemIOImpl;

public class ElevatorIOSim extends GenericMotorSubsystemIOImpl implements ElevatorIO {

    public ElevatorIOSim()
    {
        super(ElevatorConstants.kSubSysConstants, true);
    }
}
