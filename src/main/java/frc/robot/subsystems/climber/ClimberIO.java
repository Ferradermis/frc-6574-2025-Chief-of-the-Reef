package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import java.util.concurrent.atomic.DoubleAccumulator;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import org.littletonrobotics.junction.AutoLog;

// Interface for the Climber subsystem IO
// Instantiates the ClimberInputs class with the necessary inputs for the different IO layers
public interface ClimberIO {
  @AutoLog
  public static class ClimberInputs {
    //public MutAngle climberAngle = Degrees.mutable(0);
    public double climberAngle = 0;
    public MutAngularVelocity climberAngularVelocity = DegreesPerSecond.mutable(0);
    //public MutAngle climberSetPoint = Degrees.mutable(0);
    public double climberSetpoint = 0;
    public MutVoltage voltageSetPoint = Volts.mutable(0);
    public MutCurrent supplyCurrent = Amps.mutable(0);
    public MutCurrent torqueCurrent = Amps.mutable(0);
  }

  public default void setClimberTarget(double target) {}

  public default void stop() {}

  public default void setVoltage(double voltage) {}

  public default void updateInputs(ClimberInputs inputs) {}
}
