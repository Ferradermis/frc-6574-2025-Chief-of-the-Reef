package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

  /** The time in seconds from the FPGA start and the creation of this set of inputs */
  @AutoLog
  public static class PivotInputs {

    public double timestamp;

    public MutAngle pivotAngle;
    public MutAngularVelocity pivotAngularVelocity;
    public MutAngle pivotSetPoint;
    public MutVoltage voltageSetPoint;
    public MutCurrent supplyCurrent;
    public MutCurrent torqueCurrent;
  }

  public void setTarget(Angle target);

  public void updateInputs(PivotInputs inputs);
}
