// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.sim.mechanisms;

/** Add your docs here. */
public interface MotionProfiledMechanism {

    public default void updateArm(double angle)
    {}

    public default void updateElevator(double height)
    {}

    public default void updateVelocity(double velocity)
    {}
}
