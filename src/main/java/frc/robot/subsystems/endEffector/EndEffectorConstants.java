package frc.robot.subsystems.endEffector;

public class EndEffectorConstants {
  private EndEffectorConstants instance;

  private EndEffectorConstants() {}

  // If the instance is null, create a new instance
  // If an instance already exists, return the instance
  public EndEffectorConstants getInstance() {
    if (instance == null) {
      instance = new EndEffectorConstants();
    }

    return instance;
  }
}
