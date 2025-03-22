package frc.robot.subsystems.climberGate;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import org.littletonrobotics.junction.AutoLog;

// Interface for the Climber subsystem IO
// Instantiates the ClimberInputs class with the necessary inputs for the different IO layers
public interface ClimberGateIO {
  @AutoLog
  public static class ClimberGateInputs {
    //public MutAngle climberAngle = Degrees.mutable(0);
    public double angle = 0;
    public MutAngularVelocity angularVelocity = DegreesPerSecond.mutable(0);
    //public MutAngle climberSetPoint = Degrees.mutable(0);
    public double setpoint = 0;
    public MutVoltage voltageSetpoint = Volts.mutable(0);
    public MutCurrent supplyCurrent = Amps.mutable(0);
    public MutCurrent torqueCurrent = Amps.mutable(0);
    public boolean isGateDeployed = false;
  }

  public default void setClimberTarget(double target) {}

  public default void stop() {}

  public default void setVoltage(double voltage) {}

  public default void updateInputs(ClimberGateInputs inputs) {}
}
