// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.drivers;

import au.grapplerobotics.MitoCANdria;

/** Add your docs here. */
public class MitoCANdriaSensor {

    public MitoCANdriaSensor(int ID)
    {
        try (MitoCANdria mito = new MitoCANdria(ID)) {
            // Get and print USB1 current
            mito.getChannelCurrent(MitoCANdria.MITOCANDRIA_CHANNEL_USB1)
                .ifPresentOrElse(
                    current -> System.out.println("USB1 current: " + current + " A"),
                    () -> System.out.println("Couldn't get USB1 current"));

            // Get and print 5VA voltage
            mito.getChannelVoltage(MitoCANdria.MITOCANDRIA_CHANNEL_5VA)
                .ifPresentOrElse(
                    voltage -> System.out.println("5VA voltage: " + voltage + " V"),
                    () -> System.out.println("Couldn't get 5VA voltage"));

            // Enable USB2 channel
            mito.setChannelEnabled(MitoCANdria.MITOCANDRIA_CHANNEL_USB2, true);
            System.out.println("USB2 channel enabled");

            // Set ADJ channel voltage
            mito.setChannelVoltage(MitoCANdria.MITOCANDRIA_CHANNEL_ADJ, 3.3);
            System.out.println("ADJ channel voltage set to 3.3V");

            // Get and print ADJ channel setpoint
            mito.getChannelVoltageSetpoint(MitoCANdria.MITOCANDRIA_CHANNEL_ADJ)
                .ifPresentOrElse(
                    setpoint -> System.out.println("ADJ channel setpoint: " + setpoint + " V"),
                    () -> System.out.println("Couldn't get ADJ channel setpoint"));

            // Check if 5VB channel is enabled
            mito.getChannelEnabled(MitoCANdria.MITOCANDRIA_CHANNEL_5VB)
                .ifPresentOrElse(
                    enabled -> System.out.println("5VB channel enabled: " + (enabled == 1)),
                    () -> System.out.println("Couldn't check if 5VB channel is enabled"));

        } catch (Exception e) {
            System.out.println("An error occurred: " + e.getMessage());
        }


    }
}
