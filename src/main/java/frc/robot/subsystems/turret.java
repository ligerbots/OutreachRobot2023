// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//test comment hi
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;
import frc.robot.commands.TurnTurret;

public class turret extends TrapezoidProfileSubsystem {
  private final CANSparkMax m_motor;
  private final RelativeEncoder m_encoder;  
  private final SparkMaxPIDController m_PIDController;

  private static final double K_P = 0.05;
  private static final double K_I = 0.0;
  private static final double K_D = 0.0;
  private static final double K_FF = 0.0;

  // *Note: Neo reads a raw percentage of a full revolution [0, 1]
  // So this conversion factor is just how much measurement unit is one full revolution
  private final double RADIAN_PER_REVOLUTION = 2*Math.PI;

  static final double MAX_VEL_RADIAN_PER_SEC = Math.toRadians(120.0);
  static final double MAX_ACC_RADIAN_PER_SEC_SQ = Math.toRadians(30.0);

  // Gear ratio for robot
  static final double inputToOutputRatio = 1/162.5;
  //The offset for the turret. This in the end should be auto set with a limit switch
  static double turretOffset = Math.toRadians(0);

/* Turret design outline:
 * If 
 * 
 * 
*/

  //the angle it can not pass otherwise it will tare the wires off the robot UPDATE with robot
  static final double angleLimit = Math.toRadians(3);

  /** Creates a new NEOMotor. 
   * @param MOTOR_CAN_ID */
  public turret(int MOTOR_CAN_ID) {
    super(new TrapezoidProfile.Constraints(MAX_VEL_RADIAN_PER_SEC, MAX_ACC_RADIAN_PER_SEC_SQ));

    // create motor controller Sparkmax, assign CANID to m_motor, type of motor = brushelss
    m_motor = new CANSparkMax(MOTOR_CAN_ID, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();

    // assigns m_encoder to m_motor
    m_encoder = m_motor.getEncoder();

    m_encoder.setPosition(0.0);

    // Set the position conversion factor, factors in the reduction.
    m_encoder.setPositionConversionFactor(RADIAN_PER_REVOLUTION*inputToOutputRatio);

    m_PIDController = m_motor.getPIDController();
    m_PIDController.setP(K_P);
    m_PIDController.setI(K_I);
    m_PIDController.setD(K_D);
    m_PIDController.setFF(K_FF);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("encoderReading", Math.toDegrees(getAngle()));
    SmartDashboard.putNumber("turretOffsetInRadians", getAngle());
  }

  public double getAngle(){
    return m_encoder.getPosition();
  }

  private void setSpeed(double speed){
    m_motor.set(speed);
  }

  private void setAngle(double angle){
    m_PIDController.setReference(angle, ControlType.kPosition, 0, K_FF); 

  }

  /** Turns the turret. 
   * @param angle - must be bewteen 0 and 360 */
  public void setTurretAngle(double angle){
    if (angle >= 0 && angle <= 360) {
      setAngle(getAngle()-turretOffset);
    }
  }


  //REPLACE PITCH AND YAW WITH BACK AND FORTH
  /** Converts magnitudes in the x and y direction to an angle
   * @param pitch - the getY of the joystick
   * @param yaw - the getX of the joystick */
  public double convertXboxToAngle(double pitch, double yaw) {
    //pitch is negitive when fowards and yaw is negitive when left
    final double adjustedPitch = -pitch; //negitve when fowards, odd
    final double adjustedYaw = yaw;

    //If you want to understand how it works I made a graph on desmos: https://www.desmos.com/calculator/yifkhasuyr
    //CONVERT 360 TO RADIANS
    // Look up atan2()
    if (Math.copySign(1, adjustedPitch) == 1) {
      return Math.atan(adjustedYaw/adjustedPitch) % 360;
    } else {
      return (Math.atan(adjustedYaw/adjustedPitch)+180) % 360;
    }
  }

  /** 
   * @return the value you need to put in the turretOffset varable in Radians
  */
  public double getTurretOffsetAngleWhileAtZero() {
    return getAngle();
  }

  @Override
  protected void useState(State state) {
    // TODO Auto-generated method stub
    setAngle(state.position);
  }

  // set the angle to angles in radians
  private void setAngleGoal(double angle){
    super.setGoal(new State(angle, 0.0));
  }
}