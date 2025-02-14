package frc.robot.subsystems.climber;

public class ClimberConstants {
  private ClimberConstants instance;

  private ClimberConstants() {}

  // If the instance is null, create a new instance
  // If an instance already exists, return the instance
  public ClimberConstants getInstance() {
    if (instance == null) {
      instance = new ClimberConstants();
    }
    return instance;
  }
}
