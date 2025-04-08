package frc.robot.subsystems.Claw.ClawRollerLaserCAN;

import static edu.wpi.first.units.Units.Meters;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.GenericLaserCANSubsystem.GenericLaserCANSubsystem;

public class ClawRollerLaserCAN extends GenericLaserCANSubsystem {

    private Debouncer validDebouncer = new Debouncer(2, DebounceType.kRising);

    public Trigger validMeasurement =
        new Trigger(
            () -> validDebouncer.calculate(io.getValidStatus()));

    public Trigger triggered =
        new Trigger(() -> this.inputs.distance != null ? super.inputs.distance.in(Meters) <= 0.05
            : false);

    public ClawRollerLaserCAN(ClawRollerLaserCANIO io)
    {
        super(ClawRollerLaserCANConstants.kSubSysConstants.kName, io);
    }

    @Override
    public void periodic()
    {
        super.periodic();
        SmartDashboard.putBoolean("Intake Fallback Active", !validMeasurement.getAsBoolean());
        Logger.recordOutput("ClawRollerLaserCAN/Fallback Active", validMeasurement.getAsBoolean());
    }
}
