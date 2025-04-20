// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;

/** Add your docs here. */
public class TuneableProfiledPID extends ProfiledPIDController {

    // Instance name for tagging log values
    String name;

    // Tunable numbers
    private LoggedTunableNumber kP, kI, kD, maxV, maxA;

    public TuneableProfiledPID(String name, double kP, double kI,
        double kD, double maxV, double maxA)
    {
        this(name, kP, kI, kD, maxV, maxA, Constants.loopPeriodSecs);
    }

    public TuneableProfiledPID(String name, double kP, double kI,
        double kD, double maxV, double maxA, double period)
    {
        super(kP, kI, kD, new TrapezoidProfile.Constraints(maxV, maxA), period);

        this.name = name;

        // Tunable numbers for PID and motion gain constants
        this.kP = new LoggedTunableNumber(name + "/kP", kP);
        this.kI = new LoggedTunableNumber(name + "/kI", kI);
        this.kD = new LoggedTunableNumber(name + "/kD", kD);

        this.maxV = new LoggedTunableNumber(name + "/maxV", maxV);
        this.maxA = new LoggedTunableNumber(name + "/maxA", maxA);
    }

    public void updatePID()
    {
        // If changed, update controller constants from Tuneable Numbers
        if (kP.hasChanged(hashCode())
            || kI.hasChanged(hashCode())
            || kD.hasChanged(hashCode())) {
            this.setPID(kP.get(), kI.get(), kD.get());
        }

        if (maxV.hasChanged(hashCode())
            || maxA.hasChanged(hashCode())) {
            this.setConstraints(new TrapezoidProfile.Constraints(maxV.get(), maxA.get()));
        }
    }

}
