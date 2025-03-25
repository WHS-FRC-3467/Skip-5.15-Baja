package frc.robot.subsystems.Claw.ClawRollerLaserCAN;

import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import frc.robot.Ports;
import frc.robot.subsystems.GenericLaserCANSubsystem.GenericLaserCANSubsystemConstants;

public class ClawRollerLaserCANConstants {
    public static final GenericLaserCANSubsystemConstants kSubSysConstants =
        new GenericLaserCANSubsystemConstants();

    static {
        kSubSysConstants.kName = "ClawRollerLaserCAN";
        kSubSysConstants.laserCANDeviceId = Ports.CLAW_LASERCAN;
        kSubSysConstants.rangingMode = RangingMode.SHORT;
        kSubSysConstants.regionOfInterest = new RegionOfInterest(8, 8, 16, 4);
        kSubSysConstants.timingBudget = TimingBudget.TIMING_BUDGET_20MS;
    }
}
