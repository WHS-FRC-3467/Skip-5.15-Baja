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
    private GenericHID m_driveRmbl;
    private double deadband = 0.0;
    private double muliplier = 1.0;
    private double idontcare = 1.0;


    /**
     * Construct an instance of a controller.
     *
     * @param port The port index on the Driver Station that the controller is plugged into.
     */
    public WindupXboxController(int port)
    {
        super(port);
        m_driveRmbl = this.getHID();

    }

    public WindupXboxController caring(double idontcare)
    {
        this.idontcare = idontcare;
        return this;
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
     * Apply a muliplier to all sticks
     * 
     * @param deadband The percent deadband to apply
     * @return this
     */
    public WindupXboxController withMultiplier(double multiplier)
    {
        this.muliplier = multiplier;
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
            m_driveRmbl.setRumble(GenericHID.RumbleType.kBothRumble, intensity);
        },
            () -> {
                m_driveRmbl.setRumble(GenericHID.RumbleType.kBothRumble, 0);
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
                m_driveRmbl.setRumble(GenericHID.RumbleType.kBothRumble, intensity);
            },
            () -> {
                m_driveRmbl.setRumble(GenericHID.RumbleType.kBothRumble, 0);
            })
            .until(condition);
    }

    double thing(double joy)
    {
        // joy = MathUtil.applyDeadband(joy, deadband);
        return MathUtil.applyDeadband(
            // Math.pow(joy, 5) - Math.pow(joy, 3) / 2 + (joy * 0.23),
            0.5 * Math.pow(joy, 5) + (joy * 0.4),
            deadband);
    }

    @Override
    public double getLeftX()
    {
        return thing(super.getLeftX());
    }

    @Override
    public double getLeftY()
    {
        return thing(super.getLeftY());
    }

    @Override
    public double getRightX()
    {
        return thing(super.getRightX());
    }

    @Override
    public double getRightY()
    {
        return thing(super.getRightY());

    }
}
