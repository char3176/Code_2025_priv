package team3176.robot.constants;

public class SuperStructureConstants {

  /** How many amps the arm motor can use. */

  /** Percent output to run the arm up/down at */

  public static final double CLIMB_kP = 0.006;
  public static final double CLIMB_kI = 0; // .0025
  public static final double CLIMB_kD = 0; // .001
  public static final double CLIMB_kg = 0.2;
  public static final double CLIMB_TOLERANCE = 3;
  public static final double CLIMB_ZERO_POS = 0;
  public static final double CLIMB_MAXRETRACT_POS = CLIMB_ZERO_POS - 20;
  public static final double CLIMB_DEPLOY_POS = 160 + CLIMB_ZERO_POS;
  public static final double CLIMB_MAXDEPLOY_POS = 200 + CLIMB_ZERO_POS;
  public static final double CLIMB_SIM_OFFSET = 63 + CLIMB_ZERO_POS;

  public static final double ARM_OUTPUT_POWER = 1;
  public static final int ARM_CURRENT_LIMIT_A = 15;
  public static final double ARM_kP = 0.006;
  public static final double ARM_kI = 0; // .0025
  public static final double ARM_kD = 0; // .001
  public static final double ARM_kg = 0.2;
  public static final double ARM_TOLERANCE = 3;
  public static final double ARM_ZERO_POS = 0.39;
  public static final double ARM_TOP_POS = 75 + ARM_ZERO_POS;
  public static final double ARM_SIM_OFFSET = 63 + ARM_ZERO_POS;
  public static final double ARM_ENCODER_OFFSET = 0;
  public static final double CORAL_DISTANCE = 20;

  public static final double ELEVATORLEADERj_kP = 0.006;
  public static final double ELEVATORLEADER_kP = 0.006;
  public static final double ELEVATORLEADER_kI = 0; // .0025
  public static final double ELEVATORLEADER_kD = 0; // .001
  public static final double ELEVATORLEADER_kg = 0.2;
  public static final double ELEVATORLEADER_TOLERANCE = 3;
  public static final double ELEVATORLEADER_ZERO_POS = 0.0;
  public static final double ELEVATORLEADER_TOP_POS = 75 + ELEVATORLEADER_ZERO_POS;
  public static final double ELEVATORLEADER_SIM_OFFSET = 63 + ELEVATORLEADER_ZERO_POS;
  public static final double ELEVATORLEADER_L0_POS = 0 + ELEVATORLEADER_ZERO_POS;
  public static final double ELEVATORLEADER_L1_POS = 25 + ELEVATORLEADER_ZERO_POS;
  public static final double ELEVATORLEADER_L2_POS = 50 + ELEVATORLEADER_ZERO_POS;
  public static final double ELEVATORLEADER_L3_POS = 75 + ELEVATORLEADER_ZERO_POS;
  public static final double ELEVATORLEADER_L4_POS = 107 + ELEVATORLEADER_ZERO_POS;
  public static final double ELEVATORLEADER_HF_POS = 30 + ELEVATORLEADER_ZERO_POS;

  /*
  public static final double ELEVATORRIGHT_kP = 0.006;
  public static final double ELEVATORRIGHT_kI = 0; // .0025
  public static final double ELEVATORRIGHT_kD = 0; // .001
  public static final double ELEVATORRIGHT_kg = 0.2;
  public static final double ELEVATORRIGHT_TOLERANCE = 3;
  public static final double ELEVATORRIGHT_ZERO_POS = 0.39;
  public static final double ELEVATORRIGHT_TOP_POS = 75 + ELEVATORRIGHT_ZERO_POS;
  public static final double ELEVATORRIGHT_SIM_OFFSET = 63 + ELEVATORRIGHT_ZERO_POS;
  */

  public static final double INTAKE_PIVOT_kP = 0.006;
  public static final double INTAKE_PIVOT_kI = 0; // .0025
  public static final double INTAKE_PIVOT_kD = 0; // .001
  public static final double INTAKE_PIVOT_kg = 0.2;
  public static final double INTAKE_PIVOT_TOLERANCE = 3;
  public static final double INTAKE_PIVOT_ZERO_POS = 0.39;
  public static final double INTAKE_PIVOT_PICKUP_POS = 70 + INTAKE_PIVOT_ZERO_POS;
  public static final double INTAKE_PIVOT_CARRY_POS = INTAKE_PIVOT_ZERO_POS;
  public static final double INTAKE_PIVOT_SIM_OFFSET = 75 + INTAKE_PIVOT_ZERO_POS;
  /*   public static final int INTAKE_LASERCAN_DIST_TO_NOTE = 200;
   */
  public static final double INTAKE_ROLLER_kP = 0.006;
  public static final double INTAKE_ROLLER_kI = 0; // .0025
  public static final double INTAKE_ROLLER_kD = 0; // .001
  public static final double INTAKE_ROLLER_kg = 0.2;
  public static final double INTAKE_ROLLER_TOLERANCE = 3;

}
