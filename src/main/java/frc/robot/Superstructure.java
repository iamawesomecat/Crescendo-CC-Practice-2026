package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.FlywheelsConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;

public class Superstructure {
  private final Indexer indexer;
  private final Flywheels flywheels;
  private final Pivot pivot;

  public Superstructure(Indexer indexer, Flywheels flywheels, Pivot pivot) {
    this.indexer = indexer;
    this.flywheels = flywheels;
    this.pivot = pivot;
  }

  public Command stationIntake() {
    if (indexer != null) {
      return Commands.parallel(
          indexer.setPowerCommand(IndexerConstants.INTAKING_POWER),
          pivot.setPositionCommand(PivotConstants.PIVOT_STATION_POSITION));
    } else {
      return Commands.none().withName("indexer disabled");
    }
  }

  public Command subwooferShot() {
    if (flywheels != null && indexer != null) {
      return Commands.sequence(
          Commands.parallel(
              pivot.setPositionCommand(PivotConstants.SUBWOOFER_ANGLE),
              flywheels
                  .setVelocityCommand(
                      FlywheelsConstants.SUBWOOFER_RPM_LEFT, FlywheelsConstants.SUBWOOFER_RPM_RIGHT)
                  .until(
                      () ->
                          flywheels.atTargetVelocity(
                              FlywheelsConstants.SUBWOOFER_RPM_LEFT,
                              FlywheelsConstants.SUBWOOFER_RPM_RIGHT,
                              FlywheelsConstants.VELOCITY_TOLERANCE_RPM))),
          indexer.setPowerCommand(IndexerConstants.FEED_POWER));
    } else {
      return Commands.none().withName("flywheels or indexer disabled");
    }
  }
}
