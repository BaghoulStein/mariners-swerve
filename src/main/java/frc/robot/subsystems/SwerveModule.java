package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MarinersLib.TalonController;
import frc.robot.MarinersLib.TalonController.motorType;

public class SwerveModule extends SubsystemBase {
  private static final double SPIN_RATIO = 6.061;
  private static final double STEER_RATIO = 2.5;
  private static final double WHEEL_PERIMETER = 0.31918;
  
  public static final double MAX_SPEED = 3;

  private TalonController spin;
  private TalonController steer;

  private double steer_rotations;
  private SwerveModuleState target;
  private SwerveModuleState state;

  public SwerveModule(int spin, int steer) {
    this.spin = new TalonController(spin, motorType.Falcon500, FeedbackDevice.IntegratedSensor);
    this.steer = new TalonController(steer, motorType.SevenSevenFive, FeedbackDevice.QuadEncoder, 15);

    reset();
    CommandScheduler.getInstance().registerSubsystem(this);;
  }

  public void reset() {
    spin.reset();
    steer.reset();

    steer_rotations = 0;
    target = new SwerveModuleState();
    state = new SwerveModuleState();
  }

  @Override
  public void periodic() {
    steer_rotations = steer.getRotations() / STEER_RATIO;
    state.angle = Rotation2d.fromDegrees(steer_rotations * 360.0);
    state.speedMetersPerSecond = spin.getRPS() * WHEEL_PERIMETER / SPIN_RATIO;
  }

  public void set(double angle, double speed){
    target.angle = Rotation2d.fromDegrees(angle);
    target.speedMetersPerSecond = speed;
    set(target);
  }

  public void set(SwerveModuleState target){
    target = SwerveModuleState.optimize(target, state.angle);
    this.target = target;

    spin.setVelocity(meterPerSecToRPS(target.speedMetersPerSecond));
    steer.setPosition(degreesToRotations(minChangeInSteerAngle(target.angle.getDegrees())));
  }

  public void lockPosition(){
    spin.lockPosition();
  }
 
  public void stop() {
    spin.set(ControlMode.PercentOutput, 0);
    steer.set(ControlMode.PercentOutput, 0);
  }

  public SwerveModuleState getState() {
    return state;
  }

  public double getAngle(){
    return state.angle.getDegrees();
  }

  public double getSpeed(){
    return state.speedMetersPerSecond;
  }

  private double meterPerSecToRPS(double speed) { 
    return speed * SPIN_RATIO / WHEEL_PERIMETER;
  }

  private double degreesToRotations(double angle) { 
    return angle * STEER_RATIO / 360.0;
  }

  private double minChangeInSteerAngle(double angle) {
    double full_rotations = (int)steer_rotations;
    double close_angle = angle + 360.0 * full_rotations;
    double angle_plus = close_angle + 360;
    double angle_minus = close_angle - 360;

    double minAngle = close_angle;
    if(Math.abs(minAngle - getAngle()) > Math.abs(angle_plus - getAngle())) minAngle = angle_plus;
    if(Math.abs(minAngle - getAngle()) > Math.abs(angle_minus - getAngle())) minAngle = angle_minus;

    return minAngle;
  }

  public void DEBUG() {
    SmartDashboard.putNumber("angle", getAngle());
    SmartDashboard.putNumber("velocity", getSpeed());
  }
}
