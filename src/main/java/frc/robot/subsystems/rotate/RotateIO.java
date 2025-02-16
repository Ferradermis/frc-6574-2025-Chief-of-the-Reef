package frc.robot.subsystems.rotate;

import org.littletonrobotics.junction.AutoLog;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;

// Interface for the Rotate subsystem IO
// Instantiates the RotateInputs class with the necessary inputs for the different IO layers
public interface RotateIO {
    @AutoLog
    public static class RotateInputs {
        public MutAngle angle = Degrees.mutable(0);
        public MutAngularVelocity angularVelocity = DegreesPerSecond.mutable(0);
        public MutAngle setpoint = Degrees.mutable(0);
        public MutVoltage voltageSetpoint = Volts.mutable(0);
        public MutCurrent supplyCurrent = Amps.mutable(0);
        public MutCurrent torqueCurrent = Amps.mutable(0);
    }

    public default void setTarget(Angle target) {}

    public default void updateInputs(RotateInputs inputs) {}

    public default void stop() {}

    public default void setVoltage(double voltage) {}

    public default RotateConstants getConstants() {
        return new RotateConstants();
    }
}
