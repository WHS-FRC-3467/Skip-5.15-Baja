package frc.robot;

import frc.robot.util.drivers.CanDeviceId;

public class Ports {
    /*
     * LIST OF CHANNEL AND CAN IDS
     */

    public static final CanDeviceId MITOCANDRIA = new CanDeviceId(15, "rio");
    public static final CanDeviceId CLAW_LASERCAN = new CanDeviceId(16, "rio");
    public static final CanDeviceId CLAW_ROLLER = new CanDeviceId(17, "rio");

    public static final CanDeviceId ARM_CANCODER = new CanDeviceId(18, "rio");
    public static final CanDeviceId ARM_MAIN = new CanDeviceId(19, "rio");

    public static final CanDeviceId ELEVATOR_CANDLE = new CanDeviceId(20, "rio");

    public static final CanDeviceId RAMP_LASERCAN = new CanDeviceId(21, "rio");
    public static final CanDeviceId OVERHEAD_LASERCAN = new CanDeviceId(26, "rio");

    public static final CanDeviceId ELEVATOR_MAIN = new CanDeviceId(22, "rio"); // Top Kraken
    public static final CanDeviceId ELEVATOR_FOLLOWER = new CanDeviceId(23, "rio"); // Bottom Kraken

    public static final CanDeviceId CLIMBER = new CanDeviceId(24, "Drivetrain");

    public static final CanDeviceId PDH = new CanDeviceId(25, "rio");

    public static final CanDeviceId TOUNGE = new CanDeviceId(30, "rio");

}
