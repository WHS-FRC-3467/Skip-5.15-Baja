package frc.robot.subsystems.FrontRightLaserCAN;

import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.GenericLaserCANSubsystem.GenericLaserCANSubsystem;

public class FrontRightLaserCAN extends GenericLaserCANSubsystem {

    private Debouncer validDebouncer = new Debouncer(2, DebounceType.kRising);

    public Trigger validMeasurement =
        new Trigger(
            () -> validDebouncer.calculate(io.getValidStatus()));

    public Trigger triggered =
        new Trigger(() -> this.io.getConnectedStatus() ? (this.inputs.distance != null
            ? super.inputs.distance.in(Meters) <= 0.05
            : true) : true); // Return TRUE if not connected

    public FrontRightLaserCAN(FrontRightLaserCANIO io)
    {
        super(FrontRightLaserCANConstants.kSubSysConstants.kName, io);
    }
}
