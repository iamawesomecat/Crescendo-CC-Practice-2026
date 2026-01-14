package frc.robot.subsystems.pivot;

public class PivotConstants {
  public static final double GEAR_RATIO = 112.13;

  // Pivot position limits in rotations, from encoder that is 1:1 with pivot
  public static final double PIVOT_POSITION_MIN = 0.0;
  public static final double PIVOT_POSITION_MAX = 0.5;

  public static final double PIVOT_HOME_POSITION = 0.0;
  public static final double PIVOT_POSITION_TOLERANCE = 0.01;
  public static final double PIVOT_STATION_POSITION = 0.25;

  public static final double MASS_KG = 0;

  public static final double MOI = 25.0;

  public static final double LENGTH_METERS = 0.736600;

  public static final double MIN_ANGLE_RADIANS = 0;

  public static final double MAX_ANGLE_RADIANS = 1 / 2 * Math.PI;

  public static final double SIMULATION_STARTING_ANGLE_RADS = 0;

  public static final boolean SIMULATE_GRAVITY = true;

  public static final double MEAUSREMENT_STD_DEV_RADS = 0;

  public static final double SUBWOOFER_ANGLE = 0.25;
}
