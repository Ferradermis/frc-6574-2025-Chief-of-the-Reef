package frc.robot.subsystems.endEffector;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

// Interface for the EndEffector subsystem IO
// Instantiates the EndEffectorInputs class with the necessary inputs for the different IO layers
public interface EndEffectorIO {
  @AutoLog
  public static class EndEffectorInputs {
    public MutAngularVelocity angularVelocity = DegreesPerSecond.mutable(0);
    public MutVoltage voltageSetpoint = Volts.mutable(0);
    public MutVoltage voltage = Volts.mutable(0);
    public MutCurrent supplyCurrent = Amps.mutable(0);
    public MutCurrent statorCurrent = Amps.mutable(0);
  }

  public default void setTarget(Voltage setpoint) {}

  public default void updateInputs(EndEffectorInputs inputs) {}

  public default void stop() {}
}
