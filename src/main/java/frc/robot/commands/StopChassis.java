package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class StopChassis extends CommandBase {
  private Chassis chassis = Chassis.getInstance();
  public StopChassis() {
    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    chassis.stop();
  }

  @Override
  public void execute() {
    chassis.stop();
  }

  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
