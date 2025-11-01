// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {

  // Constants to limit the turret rotation speed
  private static final double MAX_VEL_ROT_PER_SEC = 0.5;
  private static final double MAX_ACC_ROT_PER_SEC2 = 3.0;
  private static final double ROBOT_LOOP_PERIOD = 0.02;

  private static final int CURRENT_LIMIT = 60;

  private static final double GEAR_RATIO = 1.0 / 15.0 * 18.0 / 195.0;

  // Trapezoid Profile
  private final TrapezoidProfile m_profile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(MAX_VEL_ROT_PER_SEC, MAX_ACC_ROT_PER_SEC2));
  private State m_currentState = new State();

  private Rotation2d m_goal = Rotation2d.fromDegrees(0);
  private Rotation2d m_goalClipped = Rotation2d.fromDegrees(0);

  private SparkMax m_turretMotor;

  private static final double MIN_ANGLE_DEG = -45.0;
  private static final double MAX_ANGLE_DEG = 45.0;

  private static final double K_P = 15.0;
  private static final double K_I = 0.0;
  private static final double K_D = 0.0;

  private final SparkClosedLoopController m_controller;

  /** Creates a new Turret. */
  public Turret() {
    m_turretMotor = new SparkMax(Constants.TURRET_MOTOR_CAN_ID, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(false);
    // always set a current limit
    config.smartCurrentLimit(CURRENT_LIMIT);
    config.idleMode(IdleMode.kBrake);
    config.encoder.positionConversionFactor(GEAR_RATIO);
    config.encoder.velocityConversionFactor(GEAR_RATIO);

    // set up the PID for MAX Motion
    config.closedLoop.p(K_P).i(K_I).d(K_D);

    config.closedLoop.outputRange(-1, 1);
    config.closedLoop.positionWrappingEnabled(false); // don't treat it as a circle
    // config.closedLoop.positionWrappingInputRange(0,1.0);

    m_turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // controller for PID control
    m_controller = m_turretMotor.getClosedLoopController();

    m_turretMotor.getEncoder().setPosition(0);
    SmartDashboard.putNumber("turret/testAngle", 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_goal = Rotation2d.fromDegrees(SmartDashboard.getNumber("turret/testAngle", 0));
    m_goalClipped = limitAngle(m_goal);

    State goalState = new State(m_goalClipped.getRotations(), 0);

    // Tra pezoid Profile
    m_currentState = m_profile.calculate(ROBOT_LOOP_PERIOD, m_currentState, goalState);

    m_controller.setReference(m_currentState.position, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);

    SmartDashboard.putNumber("turret/currentAngle", getAngle().getDegrees());
    SmartDashboard.putNumber("turret/goalAngle", m_goalClipped.getDegrees());
  }

  public Rotation2d limitAngle(Rotation2d angle) {
    return Rotation2d.fromDegrees(MathUtil.clamp(angle.getDegrees(), MIN_ANGLE_DEG, MAX_ANGLE_DEG));
  }

  // get the current hood angle
  // public Rotation2d getAngle() {
  //   return Rotation2d.fromRotations(m_turretMotor.getEncoder().getPosition());
  // }
}
