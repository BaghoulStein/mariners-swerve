package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private static Joystick prototype_stick = new Joystick(0);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {
    return null;
  }

  public static SwerveModuleState stateFromJoystick(){
    double y = prototype_stick.getRawAxis(0);
    double x = prototype_stick.getRawAxis(1);

    if(Math.abs(x) < 0.1) x = 0;
    if(Math.abs(y) < 0.1) y = 0;

    return new SwerveModuleState(Math.sqrt(x * x + y * y) * 0.5, new Rotation2d(x, y));
  }
}
