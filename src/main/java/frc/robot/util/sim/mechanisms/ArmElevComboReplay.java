package frc.robot.util.sim.mechanisms;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.util.sim.ElevatorSimConfiguration;

public class ArmElevComboReplay {

    MotionProfiledMechanism m_Mech;
    private final ElevatorSimConfiguration m_ElevConst;

    protected static ArmElevComboReplay m_ArmElevReplay;

    public static ArmElevComboReplay getInstance()
    {
        if (m_ArmElevReplay == null) {
            m_ArmElevReplay =
                new ArmElevComboReplay(ElevatorConstants.kSubSysConstants.kElevSimConfig);
        }
        return m_ArmElevReplay;
    }

    // Creates the replay mechanism
    private ArmElevComboReplay(final ElevatorSimConfiguration elevConst)
    {
        m_ElevConst = elevConst;

        if (m_ElevConst.kIsComboSim) {
            m_Mech = ArmElevComboMechanism.getInstance();
        } else {
            m_Mech = new MotionProfiledElevatorMechanism("Replay_Elevator");
        }
    }

    /* Update the mechanism on AdvantageScope according to its position */
    public void run(double elevatorPositionRot)
    {
        // This Works
        Logger.recordOutput("/SimMechPoses/PoseRot", elevatorPositionRot);
        Logger.recordOutput("/SimMechPoses/PoseInches", elevatorPositionRot * 1.9 * Math.PI
            * ElevatorConstants.kSubSysConstants.kSimMotorConfig.Feedback.SensorToMechanismRatio);
        // Pose3d not updating yet, TODO: fix
        // Publish Pose3d for 3D mechanism sim of 2 stage elevator
        Logger.recordOutput(
            "/SimMechPoses/Stage1/Pose3d",
            new Pose3d(0, 0, elevatorPositionRot * 1.9 * Math.PI
                * ElevatorConstants.kSubSysConstants.kSimMotorConfig.Feedback.SensorToMechanismRatio,
                new Rotation3d()));
        // TODO: Get the 1.9 value the actual diameter of the spinny thing/pulley from CAD
        Logger.recordOutput(
            "/SimMechPoses/Stage2/Pose3d",
            new Pose3d(0, 0, elevatorPositionRot * 1.9 * Math.PI
                * ElevatorConstants.kSubSysConstants.kSimMotorConfig.Feedback.SensorToMechanismRatio
                / 2, new Rotation3d()));
        SmartDashboard.putNumber("ElevatorInches", elevatorPositionRot * 1.9 * Math.PI
            * ElevatorConstants.kSubSysConstants.kSimMotorConfig.Feedback.SensorToMechanismRatio); // Creates
                                                                                                   // mech2d
                                                                                                   // in
                                                                                                   // SmartDashboard

        // TODO: what I want to do: put the entire Arm ElevComboMechanism into sim and have it
        // update in REPLAY mode only

    }
}
