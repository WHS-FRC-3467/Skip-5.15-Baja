package frc.robot.subsystems.GenericRollerSubsystem;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public abstract class GenericRollerSubsystem<G extends GenericRollerSubsystem.VoltageState>
    extends SubsystemBase {

    public interface VoltageState {

        public double getOutput(int index);
    }

    public abstract G getState();

    private final String name;
    private final int numRollers;
    private final GenericRollerSubsystemIO io;
    protected final GenericRollerIOInputsAutoLogged inputs = new GenericRollerIOInputsAutoLogged();
    private final List<Alert> disconnected = new ArrayList<Alert>();

    public GenericRollerSubsystem(String name, int numRollers, GenericRollerSubsystemIO io)
    {
        this.name = name;
        this.numRollers = numRollers;
        this.io = io;

        // Set up a disconnection Alert for each roller motor
        for (int i = 0; i < numRollers; i++) {
            this.disconnected.add(
                new Alert(name + " motor " + i + " disconnected!", Alert.AlertType.kWarning));
        }
    }

    public void periodic()
    {

        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);

        // Check and run each roller motor
        for (int i = 0; i < numRollers; i++) {
            // Check for motor disconnection
            disconnected.get(i).set(!inputs.connected[i]);
            // Run motor by voltage
            io.runVolts(i, getState().getOutput(i));
        }

        displayInfo();
    }

    private void displayInfo()
    {

        if (Constants.tuningMode) {
            Logger.recordOutput(this.name + "/Goal State", getState().toString());

            for (int i = 0; i < numRollers; i++) {
                Logger.recordOutput(this.name + "/Setpoint " + i, getState().getOutput(i));
                Logger.recordOutput(this.name + "/Appl Volt " + i, inputs.appliedVoltage[i]);
                Logger.recordOutput(this.name + "/Supply Current" + i, inputs.supplyCurrentAmps[i]);
            }
        }
    }
}
