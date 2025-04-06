// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

/** Add your docs here. */
public class TunablePIDController extends PIDController {

    // Instance name for tagging log values
    String m_name;

    // Tunable numbers
    private LoggedTunableNumber m_kP, m_kI, m_kD;

    public TunablePIDController(String name, double kP, double kI,
        double kD)
    {
        this(name, kP, kI, kD, Constants.loopPeriodSecs);
    }

    public TunablePIDController(String name, double kP, double kI,
        double kD, double period)
    {
        super(kP, kI, kD, period);

        m_name = name;

        // Tunable numbers for PID and motion gain constants
        m_kP = new LoggedTunableNumber(m_name + "/kP", kP);
        m_kI = new LoggedTunableNumber(m_name + "/kI", kI);
        m_kD = new LoggedTunableNumber(m_name + "/kD", kD);
    }

    public void updatePID()
    {
        // If changed, update controller constants from Tuneable Numbers
        if (m_kP.hasChanged(hashCode())
            || m_kI.hasChanged(hashCode())
            || m_kD.hasChanged(hashCode())) {
            this.setPID(m_kP.get(), m_kI.get(), m_kD.get());
        }
    }

}
