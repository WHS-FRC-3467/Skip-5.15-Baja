package frc.robot.subsystems.GenericLaserCANSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public abstract class GenericLaserCANSubsystem
    extends SubsystemBase {

    private final String name;
    protected final GenericLaserCANSubsystemIO io;
    protected final LaserCANIOInputsAutoLogged inputs = new LaserCANIOInputsAutoLogged();

    public GenericLaserCANSubsystem(String name, GenericLaserCANSubsystemIO io)
    {
        this.name = name;
        this.io = io;
    }

    public void periodic()
    {
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);

        displayInfo();
    }

    private void displayInfo()
    {
        if (Constants.tuningMode) {
            Logger.recordOutput(this.name + "/Current Distance", inputs.distance);
        }
    }
}
