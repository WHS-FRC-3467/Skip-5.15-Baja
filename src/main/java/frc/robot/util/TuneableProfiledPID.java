// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Add your docs here. */
public class TuneableProfiledPID extends ProfiledPIDController {

    // Instance name for tagging log values
    String m_name;

    // Tunable numbers
    private LoggedTunableNumber m_kP, m_kI, m_kD, m_maxV, m_maxA;

    public TuneableProfiledPID(String name, double kP, double kI,
        double kD, double maxV, double maxA)
    {
        super(kP, kI, kD, new TrapezoidProfile.Constraints(maxV, maxA));

        m_name = name;

        // Tunable numbers for PID and motion gain constants
        m_kP = new LoggedTunableNumber(m_name + "/kP", kP);
        m_kI = new LoggedTunableNumber(m_name + "/kI", kI);
        m_kD = new LoggedTunableNumber(m_name + "/kD", kD);

        m_maxV = new LoggedTunableNumber(m_name + "/maxV", maxV);
        m_maxA = new LoggedTunableNumber(m_name + "/maxA", maxA);

    }

    public void updatePID()
    {
        // If changed, update controller constants from Tuneable Numbers
        if (m_kP.hasChanged(hashCode())
            || m_kI.hasChanged(hashCode())
            || m_kD.hasChanged(hashCode())) {
            this.setPID(m_kP.get(), m_kI.get(), m_kD.get());
        }

        if (m_maxV.hasChanged(hashCode())
            || m_maxA.hasChanged(hashCode())) {
            this.setConstraints(new TrapezoidProfile.Constraints(m_maxV.get(), m_maxA.get()));
        }
    }

}
