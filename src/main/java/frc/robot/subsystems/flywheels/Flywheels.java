package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Hardware;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Flywheels extends SubsystemBase{
  private final TalonFX flFlywheel;
  private final TalonFX blFlywheel;
  private final TalonFX frFlywheel;
  private final TalonFX brFlywheel;
  // alerts if motor is not connected.
  private final Alert NotConnectedError =
      new Alert("ArmPivot", "Motor not connected", AlertType.kError);
  private final Debouncer notConnectedDebouncer = new Debouncer(.1, DebounceType.kBoth);
  

private final MotionMagicVelocityDutyCycle requestLeft = new MotionMagicVelocityDutyCycle(0);
private final MotionMagicVelocityDutyCycle requestRight = new MotionMagicVelocityDutyCycle(0);

  public Flywheels() {
    flFlywheel = new TalonFX(Hardware.FL_FLYWHEEL_ID);
    blFlywheel = new TalonFX(Hardware.BL_FLYWHEEL_ID);
    frFlywheel = new TalonFX(Hardware.FR_FLYWHEEL_ID);
    brFlywheel = new TalonFX(Hardware.BR_FLYWHEEL_ID);

    configureMotors();
  }

  private void configureMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    TalonFXConfigurator flConfigurator = flFlywheel.getConfigurator();
    TalonFXConfigurator blConfigurator = blFlywheel.getConfigurator();
    TalonFXConfigurator frConfigurator = frFlywheel.getConfigurator();
    TalonFXConfigurator brConfigurator = brFlywheel.getConfigurator();
    // set current limits
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // create brake mode for motors 
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // create PID gains
    config.Slot0.kP = 0.1;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kA = 0.0;
    config.Slot0.kV = 0.00;
    config.Slot0.kS = 0.0;
    config.Slot0.kG = 0.0;

    config.MotionMagic.MotionMagicAcceleration = 12000; // RPM/s
    config.MotionMagic.MotionMagicJerk = 0; // RPM/s^2

    flConfigurator.apply(config);
    blConfigurator.apply(config);

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    brConfigurator.apply(config);
    frConfigurator.apply(config);
  }


  public Command setFlywheelVelocity(double rpmLeft, double rpmRight) {
    requestLeft.Velocity = rpmLeft;
    requestRight.Velocity = rpmRight;
    return runOnce(() -> {
      flFlywheel.setControl(requestLeft);
      blFlywheel.setControl(requestLeft);
      frFlywheel.setControl(requestRight);
      brFlywheel.setControl(requestRight);
    }).withName("Set Flywheel Velocity");
  }

  public boolean atTargetVelocity(double targetRPMLeft, double targetRPMRight, double toleranceRPM) {
    double leftVelocity = (flFlywheel.getVelocity().getValueAsDouble() + blFlywheel.getVelocity().getValueAsDouble()) / 2.0;
    double rightVelocity = (frFlywheel.getVelocity().getValueAsDouble() + brFlywheel.getVelocity().getValueAsDouble()) / 2.0;

    boolean leftAtTarget = Math.abs(leftVelocity - targetRPMLeft) <= toleranceRPM;
    boolean rightAtTarget = Math.abs(rightVelocity - targetRPMRight) <= toleranceRPM;

    return leftAtTarget && rightAtTarget;
  }

  public Trigger atTargetVelocityTrigger(double targetRPMLeft, double targetRPMRight, double toleranceRPM) {
    return new Trigger(() -> atTargetVelocity(targetRPMLeft, targetRPMRight, toleranceRPM));
  }
}
