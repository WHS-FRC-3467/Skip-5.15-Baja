// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

/** Add your docs here. */
public class TunablePIDController extends PIDController {

    // Instance name for tagging log values
    String name;

    // Tunable numbers
    private LoggedTunableNumber kP, kI, kD;

    public TunablePIDController(String name, double kP, double kI,
        double kD)
    {
        this(name, kP, kI, kD, Constants.loopPeriodSecs);
    }

    public TunablePIDController(String name, double kP, double kI,
        double kD, double period)
    {
        super(kP, kI, kD, period);

        this.name = name;

        // Tunable numbers for PID and motion gain constants
        this.kP = new LoggedTunableNumber(name + "/kP", kP);
        this.kI = new LoggedTunableNumber(name + "/kI", kI);
        this.kD = new LoggedTunableNumber(name + "/kD", kD);
    }

    public void updatePID()
    {
        // If changed, update controller constants from Tuneable Numbers
        if (kP.hasChanged(hashCode())
            || kI.hasChanged(hashCode())
            || kD.hasChanged(hashCode())) {
            this.setPID(kP.get(), kI.get(), kD.get());
        }
    }

}
