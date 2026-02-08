package frc.robot.Constants;

public class canIDConstants {
  /* CAN loop */
  public static final String canivore = "canivore";
  public static final String rio = "rio";

  /* Kinematics */
  public static final int pigeon = 1;

  /* Swerve: FL, FR, BL, BR */
  public static final int[] driveMotor = {16, 3, 13, 18};
  public static final int[] steerMotor = {6, 5, 17, 20};
  public static final int[] CANcoder = {30, 31, 25, 9};

  /* End Effector */
  public static final int algaeMotor = 0;
  public static final int coralMotor = 36;
  public static final int pivotMotor = 33;

  /* Elevator */
  public static final int elevatorMotor1 = 21;
  public static final int elevatorMotor2 = 34;

  /* Pivot */
  public static final int leftPivotMotor1 = 7;
  public static final int leftPivotMotor2 = 4;
  public static final int rightPivotMotor1 = 2;
  public static final int rightPivotMotor2 = 14;

  /* Shooter */
  public static final int leftShooterMotor = 0; // TODO: Set correct CAN ID
  public static final int rightShooterMotor = 0; // TODO: Set correct CAN ID
}
