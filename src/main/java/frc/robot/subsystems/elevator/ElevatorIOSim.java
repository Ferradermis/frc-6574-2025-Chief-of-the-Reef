package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  private ElevatorFeedforward ff = new ElevatorFeedforward(0, 0, 0, 0);
  private final ProfiledPIDController controller = new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));
  private final ElevatorSim sim;

  private Voltage appliedVoltage = Volts.mutable(0);

  public ElevatorIOSim(int motorId, ElevatorSim elevatorSim) {
    sim = elevatorSim;
  }

  @Override
  public void setTarget(Distance target) {
    controller.setGoal(new State(target.in(Meters), 0));
  }

  private void updateVoltageSetpoint() {
    Distance currentDistance = Meters.of(sim.getPositionMeters());
    Voltage controllerVoltage = Volts.of(controller.calculate(currentDistance.in(Meters)));
    Voltage feedforwardVoltage = Volts.of(ff.calculate(controller.getSetpoint().position, controller.getSetpoint().velocity));
    Voltage effort = controllerVoltage.plus(feedforwardVoltage);

    runVolts(effort);
  }

  private void runVolts(Voltage volts) {
    this.appliedVoltage = volts;
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.distance.mut_replace(Distance.ofRelativeUnits(sim.getPositionMeters(), Meters));
    inputs.velocity.mut_replace(MetersPerSecond.of(sim.getVelocityMetersPerSecond()));
    inputs.setpoint.mut_replace(Meters.of(controller.getGoal().position));
    inputs.supplyCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
    inputs.torqueCurrent.mut_replace(inputs.supplyCurrent.in(Amps), Amps);
    inputs.voltageSetpoint.mut_replace(appliedVoltage);

    //Periodic
    updateVoltageSetpoint();
    sim.setInputVoltage(appliedVoltage.in(Volts));
    sim.update(0.02);
  }

  @Override
  public void stop() {
    Distance currentDistance = Distance.ofRelativeUnits(0, Meters);
    controller.reset(currentDistance.in(Meters));
    runVolts(Volts.of(0));
  }
}
