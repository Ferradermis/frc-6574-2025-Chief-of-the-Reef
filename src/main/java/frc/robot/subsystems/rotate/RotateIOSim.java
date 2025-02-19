package frc.robot.subsystems.rotate;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class RotateIOSim implements RotateIO{
    private final ProfiledPIDController controller;
    private final DCMotorSim sim;
    private Voltage appliedVoltage = Volts.mutable(0);
    private RotateConstants rotateConstants;

    // Create a new instance of the rotate simulator
    // Grabs the constants for the rotate subsystem
    public RotateIOSim(RotateConstants constants) {
        sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(1, 1), 
            DCMotor.getNEO(1),
            0.001,
            0.001);
        controller = new ProfiledPIDController(0, 0, 0, new Constraints(100000, 361));
        rotateConstants = constants;

    }

    // Updates the voltage setpoint of the simulated rotate
    private void updateVoltageSetpoint() {
        Angle currentAngle = Radians.of(sim.getAngularPositionRad());
        Voltage controllerVoltage = Volts.of(controller.calculate(currentAngle.in(Degrees)));
        Voltage effort = controllerVoltage;
        runVolts(effort);
    }

    // Sets the target voltage of the simulated arm
    private void runVolts(Voltage volts) {
        appliedVoltage = volts;
    }

    // Sets the target angle of the simulated rotate
    @Override
    public void setTarget(double target) {
        //controller.setGoal(new State(target.in(Degrees), 0));
    }

    // Sets the voltage of the simulated climber
    @Override
    public void setVoltage(double voltage) {
        System.out.println("Setting Climber Voltage");
        runVolts(Volts.of(voltage));
    }

    // Updates the inputs of the simulated rotate
    @Override
    public void updateInputs(RotateInputs inputs) {
        inputs.angle.mut_replace(
            Degrees.convertFrom(sim.getAngularPositionRad(), Radians), 
            Degrees);
        inputs.angularVelocity.mut_replace(
            DegreesPerSecond.convertFrom(sim.getAngularVelocityRadPerSec(), RadiansPerSecond),
            DegreesPerSecond);
        inputs.setpoint.mut_replace(controller.getGoal().position, Degrees);
        inputs.supplyCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
        inputs.torqueCurrent.mut_replace(inputs.supplyCurrent.in(Amps), Amps);
        inputs.voltageSetpoint.mut_replace(appliedVoltage);

        // Periodically set the input voltage
        // Update the simulator every 0.02 seconds
        updateVoltageSetpoint();
        sim.setInputVoltage(appliedVoltage.in(Volts));
        sim.update(0.02);
    }

    // Stops the simulated arm
    @Override
    public void stop() {
        Angle currentAngle = Radians.of(sim.getAngularPositionRad());
        controller.reset(currentAngle.in(Degrees));
        runVolts(Volts.of(0));
    }

    // Gets the constants of the rotate
    @Override
    public RotateConstants getConstants() {
        return rotateConstants;
    }

}
