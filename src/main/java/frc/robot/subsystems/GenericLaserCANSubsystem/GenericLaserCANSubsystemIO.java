package frc.robot.subsystems.GenericLaserCANSubsystem;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Distance;

public interface GenericLaserCANSubsystemIO {
    @AutoLog
    abstract class LaserCANIOInputs {
        public Distance distance;
    }

    default boolean getValidStatus() 
    {
        return true;
    }
    
    default void updateInputs(LaserCANIOInputs inputs)
    {}
}
