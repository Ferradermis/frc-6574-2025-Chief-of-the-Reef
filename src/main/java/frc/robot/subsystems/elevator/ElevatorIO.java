package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import org.littletonrobotics.junction.AutoLog;

// Interface for the Elevator subsystem IO
// Instantiates the ElevatorInputs class with the necessary inputs for the different IO layers
public interface ElevatorIO {
  @AutoLog
  public static class ElevatorInputs {
    public MutDistance distance = Inches.mutable(0);
    public MutLinearVelocity velocity = MetersPerSecond.mutable(0);
    public MutDistance setpoint = Inches.mutable(0);
    public MutVoltage voltageSetpoint = Volts.mutable(0);
    public MutCurrent supplyCurrent = Amps.mutable(0);
    public MutCurrent torqueCurrent = Amps.mutable(0);
    public MutDistance rightDist = Inches.mutable(0);
  }

  public default void updateInputs(ElevatorInputs inputs) {}

  public default void setTarget(Distance target) {}

  public default void stop() {}
}
