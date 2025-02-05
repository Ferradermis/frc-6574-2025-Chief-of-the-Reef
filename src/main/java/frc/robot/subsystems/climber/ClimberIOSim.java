package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.util.LoggedTunableNumber;

public class ClimberIOSim implements ClimberIO {

  private Voltage appliedVoltage = Volts.mutable(0);
  private LoggedTunableNumber servoSim;

  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          0.1, 0, 0, new Constraints(100000, 361)); // TODO: Find correct values, defaulted at 0 for now :)

  private final FlywheelSim sim;

  public ClimberIOSim() {
    sim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), 0.0028616, 1),
            DCMotor.getKrakenX60Foc(1),
            new double[] {0.001});
    servoSim = new LoggedTunableNumber("RobotState/Climber/ServoInput", 0);
  }

  @Override
  public void setClimberTarget(Angle target) {
    controller.setGoal(new State(target.in(Degrees), 0));
  }

  private void updateVoltageSetpoint() {
    // FlywheelSim needs position
    Angle currentAngle = Radians.of(sim.getOutput(0));

    Voltage controllerVoltage = Volts.of(controller.calculate(currentAngle.in(Degrees)));

    Voltage effort = controllerVoltage;

    runVolts(effort);
  }

  private void runVolts(Voltage volts) {
    this.appliedVoltage = volts;
  }

  //TODO: fix units so that they line up with the units in ClimberIOREV.java so it is easier to understand (I think??)
  @Override
  public void updateInputs(ClimberInputs input) {
    input.climberAngle.mut_replace(Degrees.convertFrom(sim.getOutput(0), Radians), Degrees);
    input.climberAngularVelocity.mut_replace(
        DegreesPerSecond.convertFrom(sim.getAngularVelocityRadPerSec(), RadiansPerSecond),
        DegreesPerSecond);
    input.climberSetPoint.mut_replace(controller.getGoal().position, Degrees);
    input.supplyCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
    input.torqueCurrent.mut_replace(input.supplyCurrent.in(Amps), Amps);
    input.voltageSetPoint.mut_replace(appliedVoltage);

    // Periodic
    updateVoltageSetpoint();
    sim.setInputVoltage(appliedVoltage.in(Volts));
    sim.update(0.02);
  }
}
