package frc.robot.util;

import java.util.function.BooleanSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class WindupXboxController extends CommandXboxController {

    Timer timer = new Timer();
    private GenericHID driveRmbl;
    private double deadband = 0.0;

    /**
     * Construct an instance of a controller.
     *
     * @param port The port index on the Driver Station that the controller is plugged into.
     */
    public WindupXboxController(int port)
    {
        super(port);
        driveRmbl = this.getHID();

    }

    /**
     * Apply a deadband to all sticks
     * 
     * @param deadband The percent deadband to apply
     * @return this
     */
    public WindupXboxController withDeadband(double deadband)
    {
        this.deadband = deadband;
        return this;
    }

    /**
     * Return a Command that rumbled both sides of the driver controller at a specific intensity for
     * a set amount of time. Intensity should be between 0 and 1
     */
    public Command rumbleForTime(double seconds, double intensity)
    {
        return Commands.startEnd(() -> {
            timer.restart();
            driveRmbl.setRumble(GenericHID.RumbleType.kBothRumble, intensity);
        },
            () -> {
                driveRmbl.setRumble(GenericHID.RumbleType.kBothRumble, 0);
            })
            .until(() -> timer.get() >= seconds);

    }

    /**
     * Return a Command that rumbled both sides of the driver controller at a specific intensity
     * until a condition is met. Intensity should be between 0 and 1
     */
    public Command rumbleUntilCondition(double intensity, BooleanSupplier condition)
    {
        return Commands.startEnd(
            () -> {
                driveRmbl.setRumble(GenericHID.RumbleType.kBothRumble, intensity);
            },
            () -> {
                driveRmbl.setRumble(GenericHID.RumbleType.kBothRumble, 0);
            })
            .until(condition);
    }

    @Override
    public double getLeftX()
    {
        return MathUtil.applyDeadband(super.getLeftX(), deadband);
    }

    @Override
    public double getLeftY()
    {
        return MathUtil.applyDeadband(super.getLeftY(), deadband);
    }

    @Override
    public double getRightX()
    {
        return MathUtil.applyDeadband(super.getRightX(), deadband);
    }

    @Override
    public double getRightY()
    {
        return MathUtil.applyDeadband(super.getRightY(), deadband);
    }
}
