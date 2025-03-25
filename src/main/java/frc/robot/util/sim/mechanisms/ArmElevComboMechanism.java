// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.sim.mechanisms;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

/** Class to draw a simulated Arm mechanism controlled by motion profiling */
public class ArmElevComboMechanism implements MotionProfiledMechanism {

    String m_SimName;
    Mechanism2d m_Mech;
    MechanismLigament2d m_Arm;
    MechanismLigament2d m_Elevator;

    protected static ArmElevComboMechanism m_ArmElevMech = null;

    public static ArmElevComboMechanism getInstance()
    {
        if (m_ArmElevMech == null) {
            m_ArmElevMech = new ArmElevComboMechanism();
        }
        return m_ArmElevMech;
    }

    private ArmElevComboMechanism()
    {

        double HEIGHT = 100; // Controls the height of the mech2d SmartDashboard
        double WIDTH = 60; // Controls width of the mech2d SmartDashboard

        m_SimName = "ArmElevatorComboSim";

        // Create a Mechanism2d display of a moving Single Jointed Arm mounted on an
        // Elevator carriage
        m_Mech = new Mechanism2d(WIDTH, HEIGHT);

        // MechanismRoot2d staticRoot = m_Mech.getRoot("Static Root", 29.5, 0);
        // @SuppressWarnings("unused")
        // MechanismLigament2d staticElevator =
        // staticRoot.append(
        // new MechanismLigament2d("Static Elevator", .5, 90, 3, new
        // Color8Bit(Color.kBlack)));

        MechanismRoot2d elevRoot = m_Mech.getRoot("Elevator Root", 30, 0);
        m_Elevator =
            elevRoot.append(
                new MechanismLigament2d("Elevator", 0.0, 90, 3, new Color8Bit(Color.kBlack)));

        MechanismLigament2d staticArm =
            m_Elevator.append(
                new MechanismLigament2d("Static Arm", .1, 30, 3, new Color8Bit(Color.kYellow)));
        m_Arm =
            staticArm
                .append(new MechanismLigament2d("Arm", .3, 90, 3, new Color8Bit(Color.kYellow)));
    }

    /** Controls the Arm segment & updates the mech2d widget in GUI. */
    public void updateArm(double degrees)
    {

        m_Arm.setAngle(degrees);

        // Publish Pose3d for 3D mechanism sim
        Logger.recordOutput(
            "/SimMechPoses/Arm/Pose3d",
            new Pose3d(
                0.2856484,
                0,
                0.225445 + (m_Elevator.getLength() * 2),
                new Rotation3d(0, Units.degreesToRadians(m_Arm.getAngle()), 0)));

        SmartDashboard.putData(m_SimName, m_Mech); // Creates mech2d in SmartDashboard
    }

    /** Controls the Elevator segment & updates the mech2d widget in GUI. */
    public void updateElevator(double position)
    {
        m_Elevator.setLength(position);

        // Publish Pose3d for 3D mechanism sim of 2 stage elevator
        Logger.recordOutput(
            "/SimMechPoses/Stage1/Pose3d",
            new Pose3d(0, 0, m_Elevator.getLength(), new Rotation3d()));
        Logger.recordOutput(
            "/SimMechPoses/Stage2/Pose3d",
            new Pose3d(0, 0, m_Elevator.getLength() * 2, new Rotation3d()));
        SmartDashboard.putData(m_SimName, m_Mech); // Creates mech2d in SmartDashboard
    }
}
