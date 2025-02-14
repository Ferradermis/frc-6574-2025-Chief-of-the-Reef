package frc.robot.subsystems.climber;

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

    public double timestamp;

    public MutAngle climberAngle;
    public MutAngularVelocity climberAngularVelocity;
    public MutAngle climberSetPoint;
    public MutVoltage voltageSetPoint;
    public MutCurrent supplyCurrent;
    public MutCurrent torqueCurrent;
  }

  public void setClimberTarget(Angle target);

  public void stop();

  public void setVoltage(double voltage);

  public void updateInputs(ClimberInputs inputs);
}
