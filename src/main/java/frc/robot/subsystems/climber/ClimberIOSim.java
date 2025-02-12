package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.LoggedTunableNumber;

public class ClimberIOSim implements ClimberIO {
  private ArmFeedforward feedforward;
  private Voltage appliedVoltage = Volts.mutable(0);
  private LoggedTunableNumber servoSim;

  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          1, 0, 0, new Constraints(100000, 361)); // TODO: Find correct values, defaulted at 0 for now :)

  private final SingleJointedArmSim sim;

  public ClimberIOSim() {
    sim = new SingleJointedArmSim(
            DCMotor.getNEO(1), 
            75, 
            SingleJointedArmSim.estimateMOI(1, Pounds.of(15).in(Kilograms)), 
            1, 
            Degrees.of(360).in(Radians), 
            Degrees.of(0).in(Radians), 
            false, 
            Degrees.of(0).in(Radians), 
            0.001,
            0.001);
    feedforward = new ArmFeedforward(0.1, 0, 0, 0);
    servoSim = new LoggedTunableNumber("RobotState/Climber/ServoInput", 0);
  }

  @Override
  public void setClimberTarget(Angle target) {
    controller.setGoal(new State(target.in(Degrees), 0));
  }

  private void updateVoltageSetpoint() {
    Angle currentAngle = Radians.of(sim.getAngleRads());
    Voltage controllerVoltage = Volts.of(controller.calculate(currentAngle.in(Degrees)));
    Voltage feedForwardVoltage = Volts.of(feedforward.calculate(controller.getSetpoint().position, controller.getSetpoint().velocity));
    Voltage effort = controllerVoltage.plus(feedForwardVoltage);
    runVolts(effort);
  }

  private void runVolts(Voltage volts) {
    this.appliedVoltage = volts;
  }

  @Override
  public void stop() {
      Angle currentAngle = Radians.of(sim.getAngleRads());
      controller.reset(currentAngle.in(Degrees));
      runVolts(Volts.of(0));
  }

  @Override
  public void setVoltage(double voltage) {
    System.out.println("Setting Climber Voltage");
    runVolts(Volts.of(voltage));
  }

  //TODO: fix units so that they line up with the units in ClimberIOREV.java so it is easier to understand (I think??)
  @Override
  public void updateInputs(ClimberInputs input) {
    input.climberAngle.mut_replace(Degrees.convertFrom(sim.getAngleRads(), Radians), Degrees);
    input.climberAngularVelocity.mut_replace(
        DegreesPerSecond.convertFrom(sim.getVelocityRadPerSec(), RadiansPerSecond),
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
