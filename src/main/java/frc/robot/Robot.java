// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import au.grapplerobotics.CanBridge;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.util.Elastic;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    private Field2d m_autoTraj = new Field2d();
    public static final double fieldLength = Units.inchesToMeters(690.876);
    public static final double fieldWidth = Units.inchesToMeters(317);
    private Command m_lastAutonomousCommand;
    private List<Pose2d> m_pathsToShow = new ArrayList<Pose2d>();
    public static final Translation2d fieldCenter =
        new Translation2d(fieldLength / 2, fieldWidth / 2);

    public Robot()
    {
        CanBridge.runTCP(); // Used for configuring LaserCANs

        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        // Set up data receivers & replay source
        switch (Constants.currentMode) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case SIM:
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger
                    .addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        // Start AdvantageKit logger
        Logger.start();

        // Check for valid swerve config
        var modules =
            new SwerveModuleConstants[] {
                    TunerConstants.FrontLeft,
                    TunerConstants.FrontRight,
                    TunerConstants.BackLeft,
                    TunerConstants.BackRight
            };
        for (var constants : modules) {
            if (constants.DriveMotorType != DriveMotorArrangement.TalonFX_Integrated
                || constants.SteerMotorType != SteerMotorArrangement.TalonFX_Integrated) {
                throw new RuntimeException(
                    "You are using an unsupported swerve configuration, which this template does not support without manual customization. The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers.");
            }
        }

        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
    }

    /** This function is called periodically during all modes. */
    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit()
    {
        if (DriverStation.isFMSAttached()) {
            Elastic.selectTab(1);
        }
        SmartDashboard.putData("Auto Path Preview", m_autoTraj);
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic()
    {
        var m_alliance = DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
        // Get currently selected command
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        // Check if is the same as the last one
        if (m_autonomousCommand != m_lastAutonomousCommand && m_autonomousCommand != null) {
            // Check if its contained in the list of our autos
            if (AutoBuilder.getAllAutoNames().contains(m_autonomousCommand.getName())) {
                // Clear the current path
                m_pathsToShow.clear();
                // Grabs all paths from the auto
                try {
                    for (PathPlannerPath path : PathPlannerAuto
                        .getPathGroupFromAutoFile(m_autonomousCommand.getName())) {
                        // Adds all poses to master list
                        m_pathsToShow.addAll(path.getPathPoses());
                    }
                } catch (IOException | ParseException e) {
                    e.printStackTrace();
                }
                // Check to see which alliance we are on Red Alliance
                if (m_alliance) {
                    for (int i = 0; i < m_pathsToShow.size(); i++) {
                        m_pathsToShow.set(i,
                            m_pathsToShow.get(i).rotateAround(fieldCenter, Rotation2d.k180deg));
                    }
                }
                // Displays all poses on Field2d widget
                m_autoTraj.getObject("traj").setPoses(m_pathsToShow);
            }
        }
        m_lastAutonomousCommand = m_autonomousCommand;

        if (!m_pathsToShow.isEmpty()) {
            var firstPose = m_pathsToShow.get(0);
            Logger.recordOutput("Alignment/StartPose", firstPose);
            SmartDashboard.putBoolean("Alignment/Translation",
                firstPose.getTranslation().getDistance(
                    m_robotContainer.m_drive.getPose().getTranslation()) <= Units
                        .inchesToMeters(4));
            SmartDashboard.putBoolean("Alignment/Rotation",
                firstPose.getRotation().minus(m_robotContainer.m_drive.getPose().getRotation())
                    .getDegrees() < 5);
        }
    }


    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit()
    {
        m_robotContainer.zeroTounge().schedule(); // Zeros the tounge on enable
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic()
    {}

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit()
    {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        if (DriverStation.isFMSAttached()) {
            Elastic.selectTab(0);
        }

        m_robotContainer.zeroTounge().schedule(); // Zeros the tounge on enable
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic()
    {}

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit()
    {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic()
    {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit()
    {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic()
    {}
}
