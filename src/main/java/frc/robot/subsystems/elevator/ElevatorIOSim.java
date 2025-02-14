package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  private ElevatorFeedforward ff = new ElevatorFeedforward(0.0, 0, 0, 0);
  private final ProfiledPIDController controller =
      new ProfiledPIDController( // TODO: all defaulted to zero until I can play with the simulator
          0.4, 0, 0, new Constraints(100000, 361));
  private final ElevatorSim sim;

  private Voltage appliedVoltage = Volts.mutable(0);

  // Create a new instance of the elevator simulator
  public ElevatorIOSim() {
    sim =
        new ElevatorSim(
            1, // TODO: some random number until I figure out how simulators work
            1, // TODO: some random number until I figure out how simulators work
            DCMotor.getNEO(2),
            0, // TODO: starting at zero until I can look at the CAD
            10, // TODO: some random number until I can look at the CAD
            false,
            0, // TODO: starting at zero until I can look at the CAD
            0.001,
            0.001);
  }

  // Sets the target distance of the simulated elevator
  @Override
  public void setTarget(Distance target) {
    controller.setGoal(new State(target.in(Meters), 1));
  }

  // Updates the voltage setpoint of the simulated elevator
  private void updateVoltageSetpoint() {
    Distance currentDistance = Meters.of(sim.getPositionMeters());
    Voltage controllerVoltage = Volts.of(controller.calculate(currentDistance.in(Meters)));
    Voltage feedforwardVoltage =
        Volts.of(
            ff.calculate(controller.getSetpoint().position, controller.getSetpoint().velocity));
    Voltage effort = controllerVoltage.plus(feedforwardVoltage);

    runVolts(effort);
  }

  // Sets the target voltage of the simulated elevator
  private void runVolts(Voltage volts) {
    this.appliedVoltage = volts;
  }

  // Updates the inputs of the simulated elevator
  @Override
  public void updateInputs(ElevatorInputs inputs) {
    inputs.distance.mut_replace(Distance.ofRelativeUnits(sim.getPositionMeters(), Meters));
    inputs.velocity.mut_replace(MetersPerSecond.of(sim.getVelocityMetersPerSecond()));
    inputs.setpoint.mut_replace(Meters.of(controller.getGoal().position));
    inputs.supplyCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
    inputs.torqueCurrent.mut_replace(inputs.supplyCurrent.in(Amps), Amps);
    inputs.voltageSetpoint.mut_replace(appliedVoltage);

    // Periodically set the input voltage
    // Update the simulator every 0.02 seconds
    updateVoltageSetpoint();
    sim.setInputVoltage(appliedVoltage.in(Volts));
    sim.update(0.02);
  }

  // Stops the motors of the simulated elevator
  @Override
  public void stop() {
    Distance currentDistance = Distance.ofRelativeUnits(0, Meters);
    controller.reset(currentDistance.in(Meters));
    runVolts(Volts.of(0));
  }
}
