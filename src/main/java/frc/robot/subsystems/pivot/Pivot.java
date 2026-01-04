package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class Pivot extends SubsystemBase {
    private final TalonFX leftPivotMotor;
    private final TalonFX rightPivotMotor;

    private final Alert NotConnectedError =
      new Alert("ArmPivot", "Motor not connected", AlertType.kError);
    private final Debouncer notConnectedDebouncer = new Debouncer(.1, DebounceType.kBoth);

      // MotionMagic voltage
    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    private double targetPos;

    public Pivot() {
        leftPivotMotor = new TalonFX(Hardware.LEFT_PIVOT_MOTOR_ID);
        rightPivotMotor = new TalonFX(Hardware.RIGHT_PIVOT_MOTOR_ID);

        configureMotors();
    }

    private void configureMotors() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        TalonFXConfigurator leftConfigurator = leftPivotMotor.getConfigurator();
        TalonFXConfigurator rightConfigurator = rightPivotMotor.getConfigurator();

        config.Feedback.FeedbackRemoteSensorID = Hardware.PIVOT_ENCODER_ID;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        config.Feedback.RotorToSensorRatio = PivotConstants.GEAR_RATIO;

        // set current limits
        config.CurrentLimits.SupplyCurrentLimit = 20;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        // create brake mode for motors
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
         
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;   

        // create PID gains
        config.Slot0.kP = 0.2;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kA = 0.0;
        config.Slot0.kV = 0.0;
        config.Slot0.kS = 0.0;
        config.Slot0.kG = 0.0;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        config.MotionMagic.MotionMagicCruiseVelocity = 5000;
        config.MotionMagic.MotionMagicAcceleration = 5000;
        config.MotionMagic.MotionMagicJerk = 100000;

        leftConfigurator.apply(config);

        // invert right motor
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightConfigurator.apply(config);
    }

    public void setTargetPosition(double position){
        targetPos = position;
        leftPivotMotor.setControl(m_request.withPosition(targetPos));
        rightPivotMotor.setControl(m_request.withPosition(targetPos));
    }
}
