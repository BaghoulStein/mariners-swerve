package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Chassis;

public class Prototype extends CommandBase {
  Chassis chassis = Chassis.getInstance();
  public Prototype() {
    addRequirements(chassis);
  }

  @Override
  public void execute(){
    SmartDashboard.putNumber("angle target", RobotContainer.stateFromJoystick().angle.getDegrees());
    SmartDashboard.putNumber("velocity target", RobotContainer.stateFromJoystick().speedMetersPerSecond);
    chassis.setPrototype(SmartDashboard.getNumber("angle target", 0), SmartDashboard.getNumber("velocity target", 0));
  }

  public void end() {
    chassis.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
