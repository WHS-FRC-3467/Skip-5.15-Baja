package frc.robot.subsystems.FrontLeftLaserCAN;

import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.GenericLaserCANSubsystem.GenericLaserCANSubsystem;

public class FrontLeftLaserCAN extends GenericLaserCANSubsystem {

    public Trigger triggered = new Trigger(() -> super.inputs.distance.in(Meters) <= 0.05);

    private Debouncer validDebouncer = new Debouncer(2, DebounceType.kRising);

    public Trigger validMeasurement =
        new Trigger(
            () -> validDebouncer.calculate(io.getValidStatus()));

    public FrontLeftLaserCAN(FrontLeftLaserCANIO io)
    {
        super(FrontLeftLaserCANConstants.kSubSysConstants.kName, io);
    }
}
