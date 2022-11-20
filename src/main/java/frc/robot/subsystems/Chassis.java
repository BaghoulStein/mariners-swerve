package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Chassis extends SubsystemBase {
  private final double CHASSIS_WIDTH = 0;
  private final double CHASSIS_LENGTH = 0;

  private final double MAX_VELOCITY = 0;
  private final double MAX_ANGULAR_VELOCITY = 0;
  private final double MAX_ACCELERATION = 0;
  private final double MAX_ANGULAR_ACCELERATION = 0;

  public enum control_mode {robot_oriented, field_oriented};

  public enum wheels {
      left_front, right_front,
      left_back, right_back
  };

  private static Chassis instance;

  private SwerveModule[] swerve_modules;
  private SwerveModuleState[] current_states;

  private SwerveModule prototype;

  private AHRS navx;

  private control_mode drive_mode;

  private SwerveDriveKinematics swerve_kinematics;

  private ChassisSpeeds speeds;
  private ChassisSpeeds field_oriented_speeds;
  private ChassisSpeeds target_speeds;
  private ChassisSpeeds field_oriented_target_speeds;

  private Rotation2d angle;

  private Chassis() {
    swerve_modules = new SwerveModule[4];
    swerve_modules[wheels.left_front.ordinal()] = new SwerveModule(Constants.LEFT_FRONT_SPIN, Constants.LEFT_FRONT_STEER);
    swerve_modules[wheels.right_front.ordinal()] = new SwerveModule(Constants.RIGHT_FRONT_SPIN, Constants.RIGHT_FRONT_STEER);
    swerve_modules[wheels.left_back.ordinal()] = new SwerveModule(Constants.LEFT_BACK_SPIN, Constants.LEFT_BACK_STEER);
    swerve_modules[wheels.right_back.ordinal()] = new SwerveModule(Constants.RIGHT_BACK_SPIN, Constants.RIGHT_BACK_STEER);

    current_states = new SwerveModuleState[4];

    prototype = new SwerveModule(12, 5);

    navx = new AHRS();

    Translation2d[] modules_position = new Translation2d[4];
    modules_position[wheels.left_front.ordinal()] = new Translation2d(CHASSIS_LENGTH / 2.0, CHASSIS_WIDTH / 2.0);
    modules_position[wheels.right_front.ordinal()] = new Translation2d(CHASSIS_LENGTH / 2.0, -CHASSIS_WIDTH / 2.0);
    modules_position[wheels.left_back.ordinal()] = new Translation2d(-CHASSIS_LENGTH / 2.0, CHASSIS_WIDTH / 2.0);
    modules_position[wheels.right_back.ordinal()] = new Translation2d(-CHASSIS_LENGTH / 2.0, -CHASSIS_WIDTH / 2.0);

    swerve_kinematics = new SwerveDriveKinematics(modules_position);

    drive_mode = control_mode.field_oriented;

    speeds = new ChassisSpeeds();
    field_oriented_speeds = new ChassisSpeeds();

    calibrate();
    reset();

    CommandScheduler.getInstance().registerSubsystem(this);
  }

  public static Chassis getInstance(){
    if(instance == null) instance = new Chassis();
    return instance;
  }

  public void calibrate() {
    navx.calibrate();
  }

  public void reset() {
    for (wheels wheel : wheels.values()) {
      swerve_modules[wheel.ordinal()].reset();
      current_states[wheel.ordinal()] = new SwerveModuleState();
    }

    prototype.reset();

    navx.reset();

    target_speeds = new ChassisSpeeds();
    field_oriented_target_speeds = new ChassisSpeeds();

    angle = new Rotation2d();
  }

  @Override
  public void periodic() {
    for(wheels wheel : wheels.values()) {
      current_states[wheel.ordinal()] = swerve_modules[wheel.ordinal()].getState();
    }

    speeds = swerve_kinematics.toChassisSpeeds(current_states);
    field_oriented_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
      speeds.omegaRadiansPerSecond, getAngleRotation()
    );

    angle = navx.getRotation2d();

    prototype.DEBUG();
  }

  public void setControlMode(control_mode drive_mode) {
    this.drive_mode = drive_mode;
  }

  public void setSpeeds(ChassisSpeeds target_speeds) {
    this.target_speeds = target_speeds;
    constraintTargetSpeedsVelocity();
    constraintTargetSpeedsAngularVelocity();
    constraintTargetSpeedsAcceleration();
    constraintTargetSpeedsAngularAcceleration();

    SwerveModuleState[] targets = getTargetModuleStates();
    for (wheels wheel : wheels.values()) {
      swerve_modules[wheel.ordinal()].set(targets[wheel.ordinal()]);
    }
  }

  public void setPrototype(double angle, double speed) {
    prototype.DEBUG();
    prototype.set(angle, speed);
  }

  public void setPrototype(SwerveModuleState state) {
    prototype.DEBUG();
    prototype.set(state);
  }

  public void stop() {
    for (wheels wheel : wheels.values()) {
      swerve_modules[wheel.ordinal()].stop();
    }

    prototype.stop();
  }

  public double getAngle() {
    return angle.getDegrees();
  }

  public Rotation2d getAngleRotation() {
    return angle;
  }

  public ChassisSpeeds getSpeeds() {
    switch(drive_mode) {
        case robot_oriented:
            return speeds;
        
        case field_oriented:
            return field_oriented_speeds;
        
        default:
            return null;
    }
  }

  private void constraintTargetSpeedsVelocity() {
    double v = Math.sqrt(target_speeds.vxMetersPerSecond * target_speeds.vxMetersPerSecond + target_speeds.vyMetersPerSecond * target_speeds.vyMetersPerSecond);

    if(v <= MAX_VELOCITY) return;

    target_speeds.vxMetersPerSecond = target_speeds.vxMetersPerSecond * MAX_VELOCITY / v;
    target_speeds.vyMetersPerSecond = target_speeds.vyMetersPerSecond * MAX_VELOCITY / v;
  }

  private void constraintTargetSpeedsAngularVelocity() {
    double omega = Math.abs(target_speeds.omegaRadiansPerSecond);
    if(omega <= MAX_ANGULAR_VELOCITY) return;

    target_speeds.omegaRadiansPerSecond = target_speeds.omegaRadiansPerSecond * MAX_ANGULAR_VELOCITY / omega;
  }

  private void constraintTargetSpeedsAcceleration() {
    double ax = (target_speeds.vxMetersPerSecond - speeds.vxMetersPerSecond) / TimedRobot.kDefaultPeriod;
    double ay = (target_speeds.vyMetersPerSecond - speeds.vyMetersPerSecond) / TimedRobot.kDefaultPeriod;
    double a = Math.sqrt(ax * ax + ay * ay);

    if(a <= MAX_ACCELERATION) return;

    target_speeds.vxMetersPerSecond = speeds.vxMetersPerSecond + 
                                        (target_speeds.vxMetersPerSecond - speeds.vxMetersPerSecond) * MAX_ACCELERATION / (Math.sqrt(2) * a);
    target_speeds.vyMetersPerSecond = speeds.vyMetersPerSecond + 
                                        (target_speeds.vyMetersPerSecond - speeds.vyMetersPerSecond) * MAX_ACCELERATION / (Math.sqrt(2) * a);
  }

  private void constraintTargetSpeedsAngularAcceleration() {
    double a = (target_speeds.omegaRadiansPerSecond - speeds.omegaRadiansPerSecond) / TimedRobot.kDefaultPeriod;

    if(a <= MAX_ANGULAR_ACCELERATION) return;

    target_speeds.omegaRadiansPerSecond = speeds.omegaRadiansPerSecond + (target_speeds.omegaRadiansPerSecond - speeds.omegaRadiansPerSecond) * MAX_ACCELERATION / a;
  }

  private SwerveModuleState[] getTargetModuleStates() {
    SwerveModuleState[] states;
    switch(drive_mode) {
        case robot_oriented:
            states = swerve_kinematics.toSwerveModuleStates(target_speeds);
            break;
        
        case field_oriented:
            field_oriented_target_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                target_speeds.vxMetersPerSecond, target_speeds.vyMetersPerSecond,
                target_speeds.omegaRadiansPerSecond, getAngleRotation()
            );
            states = swerve_kinematics.toSwerveModuleStates(field_oriented_target_speeds);
            break;
        
        default:
            states = null;
            break;
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveModule.MAX_SPEED);
    
    return states;
  }
}
