package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

public interface ElevatorIO {

    class ElevatorIOInputs {
        public MutDistance position = Inches.mutable(0);
        public MutLinearVelocity velocity = InchesPerSecond.mutable(0);

        public MutVoltage appliedVolts = Volts.mutable(0);

        public MutCurrent supplyCurrent = Amps.mutable(0);
        public MutCurrent torqueCurrent = Amps.mutable(0);
    }
    
    void updateInputs(ElevatorIOInputs inputs);
    void runVolts(Voltage volts);
    void runSetpoint(Distance position);
    void stop();

}
