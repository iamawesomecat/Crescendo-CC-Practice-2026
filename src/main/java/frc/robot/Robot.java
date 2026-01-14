// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.BuildInfo;

public class Robot extends TimedRobot {
  /** Singleton Stuff */
  private static Robot instance = null;

  public static Robot getInstance() {
    if (instance == null) instance = new Robot();
    return instance;
  }

  private final Controls Controls;
  public final Subsystems subsystems;
  private final PowerDistribution PDH;
  public final Superstructure superstructure;

  protected Robot() {
    // non public for singleton. Protected so test class can subclass

    instance = this;
    subsystems = new Subsystems();
    superstructure =
        new Superstructure(
            subsystems.indexerSubsystem, subsystems.flywheelsSubsystem, subsystems.pivotSubsystem);
    Controls = new Controls(subsystems, superstructure);
    PDH = new PowerDistribution(Hardware.PDH_ID, ModuleType.kRev);
    LiveWindow.disableAllTelemetry();
    LiveWindow.enableTelemetry(PDH);

    CommandScheduler.getInstance()
        .onCommandInitialize(
            command -> System.out.println("Command initialized: " + command.getName()));
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            (command, interruptor) ->
                System.out.println(
                    "Command interrupted: "
                        + command.getName()
                        + "; Cause: "
                        + interruptor.map(cmd -> cmd.getName()).orElse("<none>")));
    CommandScheduler.getInstance()
        .onCommandFinish(command -> System.out.println("Command finished: " + command.getName()));

    SmartDashboard.putData(CommandScheduler.getInstance());
    BuildInfo.logBuildInfo();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    subsystems.pivotSubsystem.updateMech2d();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
    if (subsystems.drivebaseSubsystem != null) {
      subsystems.drivebaseSubsystem.brakeMotors();
    }
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
