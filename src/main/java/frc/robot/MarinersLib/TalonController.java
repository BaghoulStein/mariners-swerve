package frc.robot.MarinersLib;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TalonController extends BaseTalon {
    public static enum motorType {
        Falcon500,
        SevenSevenFive
    };

    private static final ConstantsPIDF falcon_velocity_PIDF = new ConstantsPIDF(102.4, 400, 1, 25, 5.0);
    private static final ConstantsPIDF falcon_position_PIDF = new ConstantsPIDF(0.0, 409.6, 10.24, 8.096, 0.3);

    private static final ConstantsPIDF ssf_velocity_PIDF = new ConstantsPIDF(0, 0, 0, 0, 0); //new ConstantsPIDF(34.88, 136.252, 0.34, 8.515, 1.0);
    private static final ConstantsPIDF ssf_position_PIDF = new ConstantsPIDF(0.0, 150, 0.5, 2, 0.3);

    private final double ENCODER_FULL_ROTATION;

    private final ConstantsPIDF velocity_PIDF;
    private final ConstantsPIDF position_PIDF;

    private boolean velocity_at_setpoint_l;
    private boolean velocity_at_setpoint_n;
    private boolean position_at_setpoint_l;
    private boolean position_at_setpoint_n;

    private double last_velocity_target;
    private double last_position_target;

    private boolean lock;
    private double lock_position;

    /*
    ##############################################################################
            the gear_ratio is the ratio between the motor and the encoder
    ##############################################################################
    */

    public TalonController(int deviceNumber, motorType motor, FeedbackDevice encoder) {
        this(deviceNumber, motor, encoder, 1.0, false);
    }

    public TalonController(int deviceNumber, motorType motor, FeedbackDevice encoder, double gear_ratio) {
        this(deviceNumber, motor, encoder, gear_ratio, false);
    }

    public TalonController(int deviceNumber, motorType motor, FeedbackDevice encoder, boolean isInverted) {
        this(deviceNumber, motor, encoder, 1.0, isInverted);
    }

    public TalonController(int deviceNumber, motorType motor, FeedbackDevice encoder, double gear_ratio, boolean isInverted) {
        super(deviceNumber, "");

        configFactoryDefault();
        configSelectedFeedbackSensor(encoder, 0, 0);
        configPeakOutputForward(1.0);
        configPeakOutputReverse(-1.0);
        setNeutralMode(NeutralMode.Brake);
        setInverted(isInverted);

        double encoder_full_sensor_rotation;
        switch(encoder) {
            case IntegratedSensor:
                encoder_full_sensor_rotation = 2048.0;
                break;
            case CTRE_MagEncoder_Absolute:
                encoder_full_sensor_rotation = 360.0;
                break;
            default:
                encoder_full_sensor_rotation = 4096.0;
                break;
        }

        ENCODER_FULL_ROTATION = encoder_full_sensor_rotation;

        switch(motor) {
            case Falcon500:
                velocity_PIDF = ConstantsPIDF.times(falcon_velocity_PIDF, gear_ratio / encoder_full_sensor_rotation, gear_ratio);
                position_PIDF = ConstantsPIDF.times(falcon_position_PIDF, gear_ratio / encoder_full_sensor_rotation, gear_ratio);
                break;
            case SevenSevenFive:
                velocity_PIDF = ConstantsPIDF.times(ssf_velocity_PIDF, gear_ratio / encoder_full_sensor_rotation, gear_ratio);
                position_PIDF = ConstantsPIDF.times(ssf_position_PIDF, gear_ratio / encoder_full_sensor_rotation, gear_ratio);
                break;
            default:
                velocity_PIDF = new ConstantsPIDF(0, 0, 0, 0, 0);
                position_PIDF = new ConstantsPIDF(0, 0, 0, 0, 0);
                break;
        }

        reset();
    }

    public void reset(){
        set(ControlMode.PercentOutput, 0);
        velocity_at_setpoint_l = false;
        velocity_at_setpoint_n = false;
        position_at_setpoint_l = false;
        position_at_setpoint_n = false;

        last_velocity_target = 0;
        last_position_target = 0;        

        lock = false;
        lock_position = 0;

        setSelectedSensorPosition(0);
        setIntegralAccumulator(0);
        setIntegralAccumulator(0);
    }

    public void setVelocity(double rps) {
        if (rps == 0){
            lockPosition();
            return;
        }
        else lock = false;

        if(getControlMode() != ControlMode.Velocity || last_velocity_target != rps) {
            config_kF(0, velocity_PIDF.F);
            config_kP(0, velocity_PIDF.P);
            config_kI(0, 0);
            config_kD(0, velocity_PIDF.D);
            setIntegralAccumulator(0);

            velocity_at_setpoint_l = false;
            velocity_at_setpoint_n = false;

            last_velocity_target = rps;
        }

        velocity_at_setpoint_n = Math.abs(rps - getRPS()) < velocity_PIDF.TOLERANCE;
        if(!velocity_at_setpoint_l && velocity_at_setpoint_n) {
            setIntegralAccumulator(0);
            config_kI(0, velocity_PIDF.I);
        } else if(velocity_at_setpoint_l && !velocity_at_setpoint_n) {
            setIntegralAccumulator(0);
            config_kI(0, 0);
        }
        velocity_at_setpoint_l = velocity_at_setpoint_n;

        set(ControlMode.Velocity, rps * ENCODER_FULL_ROTATION / 10.0);
    }

    public void setPosition(double rotations) {
        if(getControlMode() != ControlMode.Position || last_position_target != rotations) {
            config_kF(0, position_PIDF.F);
            config_kP(0, position_PIDF.P);
            config_kI(0, 0);
            config_kD(0, position_PIDF.D);
            setIntegralAccumulator(0);

            position_at_setpoint_l = false;
            position_at_setpoint_n = false;

            last_position_target = rotations;
        }

        SmartDashboard.putNumber("rotations error", rotations - getRotations());

        position_at_setpoint_n = Math.abs(rotations - getRotations()) < position_PIDF.TOLERANCE;
        if(!position_at_setpoint_l && position_at_setpoint_n) {
            setIntegralAccumulator(0);
            config_kI(0, position_PIDF.I);
        } else if(position_at_setpoint_l && !position_at_setpoint_n) {
            setIntegralAccumulator(0);
            config_kI(0, 0);
        }
        position_at_setpoint_l = position_at_setpoint_n;

        set(ControlMode.Position, rotations * ENCODER_FULL_ROTATION);
    }

    public void lockPosition() {
        if(!lock) {
            lock = true;
            lock_position = getRotations();
        }
        setPosition(lock_position);
    }

    public double getRPS() {
        return getSelectedSensorVelocity() * 10.0 / ENCODER_FULL_ROTATION;
    }

    public double getRotations() {
        return getSelectedSensorPosition() / ENCODER_FULL_ROTATION;
    }
}
