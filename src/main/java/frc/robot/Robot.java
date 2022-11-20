package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Prototype;
import frc.robot.commands.StopChassis;
import frc.robot.subsystems.Chassis;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private Chassis chassis;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    chassis = Chassis.getInstance();
    CommandScheduler.getInstance().setDefaultCommand(chassis, new StopChassis());
    
    SmartDashboard.putNumber("angle target", 0);
    SmartDashboard.putNumber("velocity target", 0);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().setDefaultCommand(chassis, new StopChassis());
    chassis.reset();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    chassis.calibrate();
    chassis.reset();
    
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().setDefaultCommand(chassis, new Prototype());
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}
}
