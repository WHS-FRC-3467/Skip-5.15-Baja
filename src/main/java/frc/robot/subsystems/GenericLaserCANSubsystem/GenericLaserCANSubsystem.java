package frc.robot.subsystems.GenericLaserCANSubsystem;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public abstract class GenericLaserCANSubsystem<G extends GenericLaserCANSubsystem.DistanceState>
    extends SubsystemBase {

    public interface DistanceState {

        public Distance getDistance();
    }

    public abstract G getState();

    private final String name;
    protected final GenericLaserCANSubsystemIO io;
    protected final LaserCANIOInputsAutoLogged inputs = new LaserCANIOInputsAutoLogged();

    @Getter
    private boolean triggered;

    public GenericLaserCANSubsystem(String name, GenericLaserCANSubsystemIO io)
    {
        this.name = name;
        this.io = io;
    }

    public void periodic()
    {
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs); // TODO: Get this to stop crashing the replay

        triggered = inputs.distance.lte(getState().getDistance());

        displayInfo();
    }

    private void displayInfo()
    {

        if (Constants.tuningMode) {
            Logger.recordOutput(this.name + "/Trigger Distance", getState().getDistance());
            Logger.recordOutput(this.name + "/Current Distance", inputs.distance);
            Logger.recordOutput(this.name + "/Triggered", triggered);
        }
    }
}
