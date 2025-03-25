package frc.robot.subsystems.GenericLaserCANSubsystem;

import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import frc.robot.util.drivers.CanDeviceId;

/** Wrapper class for TalonFX config params (Recommend initializing in a static block!) */
public class GenericLaserCANSubsystemConstants {

    public String kName = "ERROR_ASSIGN_A_NAME";

    public CanDeviceId laserCANDeviceId;

    public RangingMode rangingMode;
    public RegionOfInterest regionOfInterest;
    public TimingBudget timingBudget;

}
