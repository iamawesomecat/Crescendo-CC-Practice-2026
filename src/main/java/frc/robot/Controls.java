// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.CompTunerConstants;

public class Controls {
  private static final int DRIVER_CONTROLLER_PORT = 0;

  private final CommandXboxController driverController;

  private final Subsystems s;
  private final Superstructure superstructure;
  // setting the max speed and other similar variables depending on which drivebase it is
  public static final double MaxSpeed = CompTunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  // kSpeedAt12Volts desired top speed
  public static double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(0.0001)
          .withRotationalDeadband(0.0001)
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public Controls(Subsystems s, Superstructure superstructure) {
    driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);
    this.s = s;
    this.superstructure = superstructure;
    configureDrivebaseBindings();
    configureSuperstructureBindings();
  }

  private Command rumble(CommandXboxController controller, double vibration, Time duration) {
    return Commands.startEnd(
            () -> controller.getHID().setRumble(RumbleType.kBothRumble, vibration),
            () -> controller.getHID().setRumble(RumbleType.kBothRumble, 0))
        .withTimeout(duration)
        .withName("Rumble Port " + controller.getHID().getPort());
  }

  private void configureDrivebaseBindings() {
    if (s.drivebaseSubsystem == null) {
      // Stop running this method
      return;
    }

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.

    // the driving command for just driving around
    s.drivebaseSubsystem.setDefaultCommand(
        // s.drivebaseSubsystem will execute this command periodically

        // applying the request to drive with the inputs
        s.drivebaseSubsystem
            .applyRequest(
                () ->
                    drive
                        .withVelocityX(getDriveX())
                        .withVelocityY(getDriveY())
                        .withRotationalRate(getDriveRotate()))
            .withName("Drive"));

    // reset the field-centric heading on back button press
    driverController
        .back()
        .onTrue(
            s.drivebaseSubsystem
                .runOnce(() -> s.drivebaseSubsystem.seedFieldCentric())
                .alongWith(rumble(driverController, 0.5, Seconds.of(0.3)))
                .withName("Reset gyro"));

    // logging the telemetry
    s.drivebaseSubsystem.registerTelemetry(logger::telemeterize);
  }

  private double getJoystickInput(double input) {
    if (driverController.leftStick().getAsBoolean()
        || driverController.rightStick().getAsBoolean()) {
      return 0; // stop driving if either stick is pressed
    }
    // Apply a deadband to the joystick input
    double deadbandedInput = MathUtil.applyDeadband(input, 0.1);
    return deadbandedInput;
  }

  // Drive for Solo controller
  // takes the X value from the joystick, and applies a deadband and input scaling
  private double getDriveX() {
    // Joystick +Y is back
    // Robot +X is forward
    return getJoystickInput(-driverController.getLeftY()) * MaxSpeed;
  }

  // takes the Y value from the joystick, and applies a deadband and input scaling
  private double getDriveY() {
    // Joystick +X is right
    // Robot +Y is left
    return getJoystickInput(-driverController.getLeftX()) * MaxSpeed;
  }

  // takes the rotation value from the joystick, and applies a deadband and input scaling
  private double getDriveRotate() {
    // Joystick +X is right
    // Robot +angle is CCW (left)
    return getJoystickInput(-driverController.getRightX()) * MaxSpeed;
  }

  private void configureSuperstructureBindings() {

    if (superstructure == null) {
      // Stop running this method
      return;
    }
    driverController.a().onTrue(superstructure.stationIntake());
    driverController.b().onTrue(superstructure.subwooferShot());
    driverController.y().onTrue(s.pivotSubsystem.setPositionCommand(0.7));
    driverController.x().onTrue(s.flywheelsSubsystem.stopCommand());
  }
}
