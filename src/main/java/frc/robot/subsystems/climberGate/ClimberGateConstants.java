package frc.robot.subsystems.climberGate;

public class ClimberGateConstants {
  private ClimberGateConstants instance;

  private ClimberGateConstants() {}

  // If the instance is null, create a new instance
  // If an instance already exists, return the instance
  public ClimberGateConstants getInstance() {
    if (instance == null) {
      instance = new ClimberGateConstants();
    }
    return instance;
  }
}
