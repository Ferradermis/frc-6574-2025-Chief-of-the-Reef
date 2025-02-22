package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private ElevatorIO elevatorIO;
  ElevatorInputsAutoLogged loggedElevator = new ElevatorInputsAutoLogged();

  // Create a new instance of the Elevator subsystem
  // Grabs the IO layer for the Elevator subsystem, could be a simulation or real hardware
  public Elevator(ElevatorIO io) {
    elevatorIO = io;
    loggedElevator.distance = Inches.mutable(0);
    loggedElevator.velocity = InchesPerSecond.mutable(0);
    loggedElevator.setpoint = Meters.mutable(0);
    loggedElevator.supplyCurrent = Amps.mutable(0);
    loggedElevator.torqueCurrent = Amps.mutable(0);
    loggedElevator.voltageSetpoint = Volts.mutable(0);

    // Set the elevator source in the visualizer
    RobotState.getInstance().setElevatorSource(loggedElevator.distance);
  }

  // Set the distance of the elevator
  public void setDistance(Distance target) {
    elevatorIO.setTarget(target);
  }

  // Create a new command to set the distance of the elevator
  public Command getNewSetDistanceCommand(double setElevator) {
    return new InstantCommand(
        () -> {
          setDistance(Inches.of(setElevator));
        },
        this);
  }

  public Command stopMotors() {
    return new InstantCommand(
        () -> {
          elevatorIO.stop();
        },
        this);
  }

  public Command resetEncoder() {
    return new InstantCommand(
      () -> {
        elevatorIO.reset();
      },
      this);
  }

  // Called periodically to update the Elevator subsystem with the new inputs and log them
  @Override
  public void periodic() {
    elevatorIO.updateInputs(loggedElevator);
    Logger.processInputs("RobotState/Elevator", loggedElevator);
  }
}
