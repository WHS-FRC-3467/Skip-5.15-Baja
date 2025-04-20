package frc.robot.subsystems.Elevator;

import frc.robot.subsystems.GenericMotorSubsystem.GenericMotorSubsystemIOImpl;

public class ElevatorIOTalonFX extends GenericMotorSubsystemIOImpl implements ElevatorIO {

    public ElevatorIOTalonFX()
    {
        super(ElevatorConstants.kSubSysConstants, false);
    }
}
