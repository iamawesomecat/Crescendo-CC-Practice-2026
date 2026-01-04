package frc.robot;

public class Hardware {
  // Add motor IDs here

  public static final int PDH_ID = 1;
  // Swerve: 1-12

  // LEDs [15-19]
  public static final int ELEVATOR_LED = 15;

  // flywheels [20-29]
  public static final int FL_FLYWHEEL_ID = 20;
  public static final int BL_FLYWHEEL_ID = 21;
  public static final int FR_FLYWHEEL_ID = 22;
  public static final int BR_FLYWHEEL_ID = 23;

  // intake [30-39]
  public static final int INDEXER_MOTOR_ID = 30;

  // pivot [40-49]
  public static final int LEFT_PIVOT_MOTOR_ID = 40;
  public static final int RIGHT_PIVOT_MOTOR_ID = 41;
  public static final int PIVOT_ENCODER_ID = 42;

  // vision
  public static final String PHOTON_IP = "10.24.12.11";
  public static final String LEFT_CAM = "Arducam_OV9782L";
  public static final String RIGHT_CAM = "Arducam_OV9281R";
}
