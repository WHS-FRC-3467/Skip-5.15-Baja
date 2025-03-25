package frc.robot.subsystems.GenericLaserCANSubsystem;

import org.littletonrobotics.junction.Logger;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import static edu.wpi.first.units.Units.*;

/**
 * Generic motion IO implementation for any motion mechanism using a TalonFX motor controller, an
 * optional follower motor, and an optional remote CANcoder encoder.
 */
public class GenericLaserCANSubsystemIOImpl implements GenericLaserCANSubsystemIO {

    private LaserCanInterface lc = null;
    private String name;
    private int tries = 0;
    private boolean hasConfiged = false;

    private Distance currentDistance;

    private final Alert failedConfig =
        new Alert("Failed to configure LaserCAN!", AlertType.kError);
    private final Alert sensorAlert =
        new Alert("Failed to get LaserCAN measurement", Alert.AlertType.kWarning);

    /*
     * Constructor
     */
    public GenericLaserCANSubsystemIOImpl(
        GenericLaserCANSubsystemConstants constants, boolean isSim)
    {
        name = constants.kName;

        lc = isSim
            ? new LaserCANSim(name)
            : new LaserCan(constants.laserCANDeviceId.getDeviceNumber());
        while (!hasConfiged && tries < 5) {
            try {
                lc.setRangingMode(constants.rangingMode);
                lc.setRegionOfInterest(constants.regionOfInterest);
                lc.setTimingBudget(constants.timingBudget);
                failedConfig.set(false);
                System.out.println("Succesfully configured " + name);
                hasConfiged = true;
            } catch (ConfigurationFailedException e) {
                System.out.println("Configuration failed for " + name + "! " + e);
                failedConfig.setText("Failed to configure " + name + "!");
                failedConfig.set(true);
                tries++;
            }
        }

    }

    public Distance getMeasurement()
    {
        Measurement measurement = lc.getMeasurement();
        if (measurement != null) {
            if (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
                sensorAlert.set(false);
                currentDistance = Millimeters.of(measurement.distance_mm);
            } else {
                sensorAlert.setText("Failed to get LaserCAN ID: " + name
                    + ", no valid measurement");
                sensorAlert.set(true);
                currentDistance = Millimeters.of(Double.POSITIVE_INFINITY);
            }
        } else {
            sensorAlert.setText("Failed to get LaserCAN ID: " + name
                + ", measurement null");
            sensorAlert.set(true);
            currentDistance = Millimeters.of(Double.POSITIVE_INFINITY);
        }
        Logger.recordOutput("LaserCANSensors/LaserCAN" + name,
            currentDistance.in(Inches));
        return currentDistance;
    }

    @Override
    public void updateInputs(LaserCANIOInputs inputs)
    {
        inputs.distance = getMeasurement();
    }
}
