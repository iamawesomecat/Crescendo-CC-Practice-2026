package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class Indexer extends SubsystemBase {

  private final TalonFX indexer;
  private final Alert NotConnectedError =
      new Alert("ArmPivot", "Motor not connected", AlertType.kError);
  private final Debouncer notConnectedDebouncer = new Debouncer(.1, DebounceType.kBoth);

  public Indexer() {
    indexer = new TalonFX(Hardware.INDEXER_MOTOR_ID);
    configureMotors();
  }

  private void configureMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    TalonFXConfigurator indexerConfigurator = indexer.getConfigurator();

    // set current limits
    config.CurrentLimits.SupplyCurrentLimit = 30;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 60;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // create brake mode for motors
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    indexerConfigurator.apply(config);
  }

  public Command setPowerCommand(double power) {
    return runOnce(
        () -> {
          indexer.set(power);
        });
  }
}
