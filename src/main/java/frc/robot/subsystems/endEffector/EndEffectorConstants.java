package frc.robot.subsystems.endEffector;

public class EndEffectorConstants {
  private EndEffectorConstants instance;

  private EndEffectorConstants() {}

  public EndEffectorConstants getInstance() {
    if (instance == null) {
      instance = new EndEffectorConstants();
    }

    return instance;
  }
}
