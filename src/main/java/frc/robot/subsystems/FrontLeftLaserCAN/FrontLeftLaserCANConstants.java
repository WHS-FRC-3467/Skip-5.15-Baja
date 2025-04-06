package frc.robot.subsystems.FrontLeftLaserCAN;

import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import frc.robot.Ports;
import frc.robot.subsystems.GenericLaserCANSubsystem.GenericLaserCANSubsystemConstants;

public class FrontLeftLaserCANConstants {
    public static final GenericLaserCANSubsystemConstants kSubSysConstants =
        new GenericLaserCANSubsystemConstants();

    static {
        kSubSysConstants.kName = "FrontLeftLaserCAN";
        kSubSysConstants.laserCANDeviceId = Ports.FRONT_LEFT_LASERCAN;
        kSubSysConstants.rangingMode = RangingMode.SHORT;
        kSubSysConstants.regionOfInterest = new RegionOfInterest(8, 8, 16, 4);
        kSubSysConstants.timingBudget = TimingBudget.TIMING_BUDGET_20MS;
    }
}
