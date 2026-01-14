package frc.robot;

import static frc.robot.Subsystems.SubsystemConstants.*;

import frc.robot.generated.CompTunerConstants;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.pivot.Pivot;

public class Subsystems {
  public static class SubsystemConstants {
    // <SUBSYSTEM>_ENABLED constants go here

    public static final boolean DRIVEBASE_ENABLED = true;
    public static final boolean FLYWHEELS_ENABLED = true;
    public static final boolean INDEXER_ENABLED = true;
    public static final boolean PIVOT_ENABLED = true;
  }

  // Subsystems go here
  public final CommandSwerveDrivetrain drivebaseSubsystem;
  public final Flywheels flywheelsSubsystem;
  public final Indexer indexerSubsystem;
  public final Pivot pivotSubsystem;

  public Subsystems() {
    // Initialize subsystems here (don't forget to check if they're enabled!)
    // Add specification for bonk, Enum? get team number?
    if (DRIVEBASE_ENABLED) {
      drivebaseSubsystem = CompTunerConstants.createDrivetrain();
    } else {
      drivebaseSubsystem = null;
    }

    if (FLYWHEELS_ENABLED) {
      flywheelsSubsystem = new Flywheels();
    } else {
      flywheelsSubsystem = null;
    }

    if (INDEXER_ENABLED) {
      indexerSubsystem = new Indexer();
    } else {
      indexerSubsystem = null;
    }

    if (PIVOT_ENABLED) {
      pivotSubsystem = new Pivot();
      System.out.println("Pivot subsystem initialized");
    } else {
      pivotSubsystem = null;
    }
  }
}
