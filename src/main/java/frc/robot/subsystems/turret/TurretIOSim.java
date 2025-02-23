package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretIOSim implements TurretIO{
    private final ProfiledPIDController controller;
    private final DCMotorSim sim;
    private Voltage appliedVoltage = Volts.mutable(0);
    private TurretConstants turretConstants;

    // Create a new instance of the turret simulator
    // Grabs the constants for the turret subsystem
    public TurretIOSim(TurretConstants constants) {
        sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(1, 1), 
            DCMotor.getNEO(1),
            0.001,
            0.001);
        controller = new ProfiledPIDController(0, 0, 0, new Constraints(100000, 361));
        turretConstants = constants;

    }

    // Updates the voltage setpoint of the simulated turret
    private void updateVoltageSetpoint() {
        Angle currentAngle = Radians.of(sim.getAngularPositionRad());
        Voltage controllerVoltage = Volts.of(controller.calculate(currentAngle.in(Degrees)));
        Voltage effort = controllerVoltage;
        runVolts(effort);
    }

    // Sets the target voltage of the simulated turret
    private void runVolts(Voltage volts) {
        appliedVoltage = volts;
    }

    // Sets the target angle of the simulated turret
    @Override
    public void setTarget(double target) {
        //controller.setGoal(new State(target.in(Degrees), 0));
    }

    // Sets the voltage of the simulated turret
    @Override
    public void setVoltage(double voltage) {
        runVolts(Volts.of(voltage));
    }

    // Updates the inputs of the simulated turret
    @Override
    public void updateInputs(TurretInputs inputs) {
        //inputs.angle.mut_replace(
            //Degrees.convertFrom(sim.getAngularPositionRad(), Radians), 
            //Degrees);
        inputs.angularVelocity.mut_replace(
            DegreesPerSecond.convertFrom(sim.getAngularVelocityRadPerSec(), RadiansPerSecond),
            DegreesPerSecond);
        //inputs.setpoint.mut_replace(controller.getGoal().position, Degrees);
        inputs.supplyCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
        inputs.torqueCurrent.mut_replace(inputs.supplyCurrent.in(Amps), Amps);
        inputs.voltageSetpoint.mut_replace(appliedVoltage);

        // Periodically set the input voltage
        // Update the simulator every 0.02 seconds
        updateVoltageSetpoint();
        sim.setInputVoltage(appliedVoltage.in(Volts));
        sim.update(0.02);
    }

    // Stops the simulated turret
    @Override
    public void stop() {
        Angle currentAngle = Radians.of(sim.getAngularPositionRad());
        controller.reset(currentAngle.in(Degrees));
        runVolts(Volts.of(0));
    }

    // Gets the constants of the turret
    @Override
    public TurretConstants getConstants() {
        return turretConstants;
    }

}
