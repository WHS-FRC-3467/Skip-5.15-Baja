package frc.robot.subsystems.Elevator;

import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemIOImpl;

public class ElevatorIOSim extends GenericMotionProfiledSubsystemIOImpl implements ElevatorIO {

    public ElevatorIOSim()
    {
        super(ElevatorConstants.kSubSysConstants, true);
    }
}
