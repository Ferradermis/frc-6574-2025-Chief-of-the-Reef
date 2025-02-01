package frc.robot.subsystems.endEffector;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class EndEffectorIOSim implements EndEffectorIO {
    /** J3_KP makes the joint motor know what speed to run at */
    public final double J3_KP = 1.0;

    public final double J3_KD = 1.0;
    public final double J3_GEARING = 1.0;

    private Voltage appliedVoltage = Volts.of(0.0);
    
    private final DCMotorSim sim;

    private final DCMotor gearbox = DCMotor.getKrakenX60Foc(1);

    private static final double flywheelkA = 0.1;
    private static final double flywheelkV = 1.0;

    private final LinearSystem<N2, N1, N2> flywheelPlant = LinearSystemId.createDCMotorSystem(flywheelkV, flywheelkA);

    public EndEffectorIOSim() {
        sim = new DCMotorSim(flywheelPlant, gearbox, 0.1, 0.1);
    }

    @Override
    public void setTarget(Voltage setpoint) {
        appliedVoltage = setpoint;
    }

    @Override
    public void updateInputs(EndEffectorInputs inputs) {
        inputs.angularVelocity.mut_replace(
        DegreesPerSecond.convertFrom(sim.getAngularVelocityRadPerSec(), RadiansPerSecond),
        DegreesPerSecond);
        inputs.supplyCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
        inputs.statorCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
        inputs.voltageSetpoint.mut_replace(appliedVoltage);
        inputs.voltage.mut_replace(Volts.of(sim.getInputVoltage()));

        // Periodic
        sim.setInputVoltage(appliedVoltage.in(Volts));
        sim.update(0.02);
    }

    @Override
    public void stop() {
    }
}
