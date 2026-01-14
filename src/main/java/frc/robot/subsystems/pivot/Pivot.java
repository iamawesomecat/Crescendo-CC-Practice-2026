package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
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
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

  private final SingleJointedArmSim pivotSim =
      new SingleJointedArmSim(
          LinearSystemId.createSingleJointedArmSystem(
              DCMotor.getKrakenX60(2), PivotConstants.MOI, PivotConstants.GEAR_RATIO),
          DCMotor.getKrakenX60(2),
          PivotConstants.GEAR_RATIO,
          PivotConstants.LENGTH_METERS,
          PivotConstants.MIN_ANGLE_RADIANS,
          PivotConstants.MAX_ANGLE_RADIANS,
          PivotConstants.SIMULATE_GRAVITY,
          PivotConstants.SIMULATION_STARTING_ANGLE_RADS);

  Mechanism2d mech = new Mechanism2d(12, 12);
  MechanismLigament2d pivotArm =
      mech.getRoot("Pivot", 5, 5)
          .append(new MechanismLigament2d("Pivot", PivotConstants.LENGTH_METERS, 0));
  private double targetPos;

  public Pivot() {
    leftPivotMotor = new TalonFX(Hardware.LEFT_PIVOT_MOTOR_ID);
    rightPivotMotor = new TalonFX(Hardware.RIGHT_PIVOT_MOTOR_ID);

    configureMotors();
    simulationInit();
    SmartDashboard.putData("Pivot Mechanism", mech);
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
    config.Slot0.kP = 100;
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
    System.out.println("Pivot motors configured");
  }

  public void simulationInit() {
    var leftPivotSimState = leftPivotMotor.getSimState();
    var rightPivotSimState = rightPivotMotor.getSimState();

    leftPivotSimState.Orientation = ChassisReference.Clockwise_Positive;
    leftPivotSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    rightPivotSimState.Orientation = ChassisReference.CounterClockwise_Positive;
    rightPivotSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    System.out.println("Pivot simulation initialized");
  }

  public void simulationPeriodic() {

    // Get the sim states
    var leftPivotSimState = leftPivotMotor.getSimState();
    var rightPivotSimState = rightPivotMotor.getSimState();

    // Set the supply voltage for the sims
    leftPivotSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    rightPivotSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    // Get the motor voltage outputs from the Talon FXs
    var leftVoltage = leftPivotSimState.getMotorVoltageMeasure();
    // Set the inputs to the pivot sims
    pivotSim.setInputVoltage(leftVoltage.in(Volts));
    // Update the sims
    pivotSim.update(0.02);

    // Update the simulated sensor readings
    leftPivotSimState.setRawRotorPosition(
        pivotSim.getAngleRads() / 2 * Math.PI * PivotConstants.GEAR_RATIO);
    rightPivotSimState.setRawRotorPosition(
        pivotSim.getAngleRads() / 2 * Math.PI * PivotConstants.GEAR_RATIO);
    leftPivotSimState.setRotorVelocity(pivotSim.getVelocityRadPerSec() * PivotConstants.GEAR_RATIO);
    rightPivotSimState.setRotorVelocity(
        pivotSim.getVelocityRadPerSec() * PivotConstants.GEAR_RATIO);
  }

  public void updateMech2d() {
    pivotArm.setAngle(Math.toDegrees(getPositionRotations()));
  }

  private double getPositionRotations() {
    return leftPivotMotor.getPosition().getValueAsDouble();
  }

  public void setTargetPosition(double position_rotations) {
    targetPos = position_rotations;
    leftPivotMotor.setControl(m_request.withPosition(targetPos));
    rightPivotMotor.setControl(m_request.withPosition(targetPos));
  }

  public Command setPositionCommand(double position_rotations) {
    return runOnce(
        () -> {
          setTargetPosition(position_rotations);
        });
  }

  public Command stopCommand() {
    return runOnce(
        () -> {
          leftPivotMotor.stopMotor();
          rightPivotMotor.stopMotor();
        });
  }
}
