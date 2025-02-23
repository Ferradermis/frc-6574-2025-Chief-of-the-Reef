package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.AutoLog;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;

// Interface for the pivot subsystem IO
// Instantiates the PivotInputs class with the necessary inputs for the different IO layers
public interface PivotIO {
    @AutoLog
    public static class PivotInputs {
        public MutAngle angle = Degrees.mutable(0);
        public double encoder = 0;
        public MutAngularVelocity angularVelocity = DegreesPerSecond.mutable(0);
        public double setpoint = 0;
        public MutVoltage voltageSetpoint = Volts.mutable(0);
        public MutCurrent supplyCurrent = Amps.mutable(0);
        public MutCurrent torqueCurrent = Amps.mutable(0);
        public double error = 0;
    }

    public default void setTarget(double target) {}

    public default void updateInputs(PivotInputs inputs) {}

    public default void stop() {}

    public default void setVoltage(double voltage) {}

    public default PivotConstants getConstants() {
        return new PivotConstants();
    }
}
