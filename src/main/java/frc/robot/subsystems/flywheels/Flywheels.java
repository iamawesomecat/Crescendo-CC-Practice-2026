package frc.robot.subsystems.flywheels;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Hardware;

public class Flywheels extends SubsystemBase {
  private final TalonFX flFlywheel;
  private final TalonFX blFlywheel;
  private final TalonFX frFlywheel;
  private final TalonFX brFlywheel;
  // alerts if motor is not connected.
  private final Alert NotConnectedError =
      new Alert("Flywheels", "Motor not connected", AlertType.kError);
  private final Debouncer notConnectedDebouncer = new Debouncer(.1, DebounceType.kBoth);

  private final MotionMagicVelocityDutyCycle requestLeft = new MotionMagicVelocityDutyCycle(0);
  private final MotionMagicVelocityDutyCycle requestRight = new MotionMagicVelocityDutyCycle(0);

  private final FlywheelSim flSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.6194175216, 1),
          DCMotor.getKrakenX60(1),
          0);
  private final FlywheelSim frSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.6194175216, 1),
          DCMotor.getKrakenX60(1),
          0);
  private final FlywheelSim blSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.6194175216, 1),
          DCMotor.getKrakenX60(1),
          0);
  private final FlywheelSim brSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.6194175216, 1),
          DCMotor.getKrakenX60(1),
          0);

  // Constructor
  public Flywheels() {
    flFlywheel = new TalonFX(Hardware.FL_FLYWHEEL_ID);
    blFlywheel = new TalonFX(Hardware.BL_FLYWHEEL_ID);
    frFlywheel = new TalonFX(Hardware.FR_FLYWHEEL_ID);
    brFlywheel = new TalonFX(Hardware.BR_FLYWHEEL_ID);

    configureMotors();
    simulationInit();
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
    config.MotionMagic.MotionMagicJerk = 1500; // RPM/s^2

    flConfigurator.apply(config);
    blConfigurator.apply(config);

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    brConfigurator.apply(config);
    frConfigurator.apply(config);
  }

  public Command setVelocityCommand(double rpmLeft, double rpmRight) {
    requestLeft.Velocity = rpmLeft;
    requestRight.Velocity = rpmRight;
    return runOnce(
            () -> {
              flFlywheel.setControl(requestLeft);
              blFlywheel.setControl(requestLeft);
              frFlywheel.setControl(requestRight);
              brFlywheel.setControl(requestRight);
            })
        .withName("Set Flywheel Velocity");
  }

  public Command stopCommand() {
    return runOnce(
            () -> {
              flFlywheel.stopMotor();
              blFlywheel.stopMotor();
              frFlywheel.stopMotor();
              brFlywheel.stopMotor();
            })
        .withName("Stop Flywheels");
  }

  public boolean atTargetVelocity(
      double targetLeftRPM, double targetRightRPM, double toleranceRPM) {
    double leftVelocity =
        (flFlywheel.getVelocity().getValueAsDouble() + blFlywheel.getVelocity().getValueAsDouble())
            / 2.0;
    double rightVelocity =
        (frFlywheel.getVelocity().getValueAsDouble() + brFlywheel.getVelocity().getValueAsDouble())
            / 2.0;

    boolean leftAtTarget = Math.abs(leftVelocity - targetLeftRPM) <= toleranceRPM;
    boolean rightAtTarget = Math.abs(rightVelocity - targetRightRPM) <= toleranceRPM;

    return leftAtTarget && rightAtTarget;
  }

  public Trigger atTargetVelocityTrigger(
      double targetLeftRPM, double targetRightRPM, double toleranceRPM) {
    return new Trigger(() -> atTargetVelocity(targetLeftRPM, targetRightRPM, toleranceRPM));
  }

  public void simulationInit() {
    var flSimState = flFlywheel.getSimState();
    var frSimState = frFlywheel.getSimState();
    var blSimState = blFlywheel.getSimState();
    var brSimState = brFlywheel.getSimState();
    flSimState.Orientation = ChassisReference.Clockwise_Positive;
    flSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    frSimState.Orientation = ChassisReference.Clockwise_Positive;
    frSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    blSimState.Orientation = ChassisReference.Clockwise_Positive;
    blSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    brSimState.Orientation = ChassisReference.Clockwise_Positive;
    brSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
  }

  public void simulationPeriodic() {

    // Get the sim states
    var flSimState = flFlywheel.getSimState();
    var frSimState = frFlywheel.getSimState();
    var blSimState = blFlywheel.getSimState();
    var brSimState = brFlywheel.getSimState();

    // Set the supply voltage for the sims
    flSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    frSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    blSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    brSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Get the motor voltage outputs from the Talon FXs
    var flVoltage = flSimState.getMotorVoltageMeasure();
    var frVoltage = frSimState.getMotorVoltageMeasure();
    var blVoltage = blSimState.getMotorVoltageMeasure();
    var brVoltage = brSimState.getMotorVoltageMeasure();

    // Set the inputs to the flywheel sims
    flSim.setInputVoltage(flVoltage.in(Volts));
    frSim.setInputVoltage(frVoltage.in(Volts));
    blSim.setInputVoltage(blVoltage.in(Volts));
    brSim.setInputVoltage(brVoltage.in(Volts));

    // Update the sims
    flSim.update(0.02);
    frSim.update(0.02);
    blSim.update(0.02);
    brSim.update(0.02);

    // Update the simulated sensor readings
    flSimState.setRotorVelocity(flSim.getAngularVelocity());
    frSimState.setRotorVelocity(frSim.getAngularVelocity());
    blSimState.setRotorVelocity(blSim.getAngularVelocity());
    brSimState.setRotorVelocity(brSim.getAngularVelocity());
  }
}
