// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Map.Entry;
import java.util.TreeMap;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;


public class Shooter extends TrapezoidProfileSubsystem {
  WPI_TalonFX m_motorLeader, m_motorFollower;
  WPI_TalonFX flup;
  TalonFXSensorCollection m_shooterEncoder;
  Servo hoodServo, turretServo;
  TreeMap<Double, Double[]> distanceLookUp = new TreeMap<Double,Double[]>() {}; //set up lookup table for ranges
  TreeMap<Double, Double> turretAngleLookup = new TreeMap<Double, Double>() {};
  DutyCycleEncoder m_dutyEncoder;

  public Vision m_vision;
  public int m_rpmAdjustment = 0;
  public int m_hoodAdjustment = 0;
  public double m_angleErrorAfterTurn = 0;

  private static final double SHOULDER_MAX_ANGLE = Math.toRadians(30.0);  
  private static final double SHOULDER_MIN_ANGLE = Math.toRadians(-65.0);

  public static final double SHOULDER_ANGLE_TOLERANCE_RADIAN = Math.toRadians(3.0);

  private static final double LEADER_CURRENT_LIMIT = 40.0;
  private static final double FOLLOW_CURRENT_LIMIT = 40.0;

  // TODO: The following constants came from the 2022 robot.
  // These need to be set for this robot.

  // All units are MKS with angles in Radians

  // Feedforward constants for the shoulder
  private static final double SHOULDER_KS = 0.182; // TODO: This may need to be tuned
  // The following constants are computed from https://www.reca.lc/arm
  private static final double SHOULDER_KG = 0.09; // V
  private static final double SHOULDER_KV = 6.60; // V*sec/rad
  private static final double SHOULDER_KA = 0.01; // V*sec^2/rad
  

  // Constants to limit the shoulder rotation speed
  private static final double SHOULDER_MAX_VEL_RADIAN_PER_SEC = Units.degreesToRadians(300.0); // 120 deg/sec
  private static final double SHOULDER_MAX_ACC_RADIAN_PER_SEC_SQ = Units.degreesToRadians(450.0); // 120 deg/sec^2

  private static final double SHOULDER_POSITION_OFFSET = 62.0/360.0;
  private static final double SHOULDER_OFFSET_RADIAN = SHOULDER_POSITION_OFFSET * 2 * Math.PI;

  // The Shoulder gear ratio is 288, but let's get it exactly.
  // private static final double SHOULDER_GEAR_RATIO = (84.0 /12.0) * (84.0 / 18.0) * (84.0 / 26.0) * (60.0 / 22.0);
  private static final double SHOULDER_GEAR_RATIO = (84.0 /12.0) * (84.0 / 18.0) * (70.0 / 40.0) * (60.0 / 22.0);

  // PID Constants for the shoulder PID controller
  // Since we're using Trapeziodal control, all values will be 0 except for P
  private static final double SHOULDER_K_P = 0.15;
  private static final double SHOULDER_K_I = 0.0;
  private static final double SHOULDER_K_D = 0.0;
  private static final double SHOULDER_K_FF = 0.0;
  private static final int kPIDLoopIdx = 0;
  private static final int kTimeoutMs = 0;
  private boolean m_coastMode = false;

  // The TalonFX, the integrated motor controller for the Falcon, uses ticks as it's noative unit.
  // There are 2048 ticks per revolution. Need to account for the gear ratio.
  private static final double SHOULDER_RADIAN_PER_UNIT = 2 * Math.PI / (2048.0 * SHOULDER_GEAR_RATIO);

  public Shooter(Vision vision, DutyCycleEncoder dutyCycleEncoder) {
    super(new TrapezoidProfile.Constraints(SHOULDER_MAX_VEL_RADIAN_PER_SEC / SHOULDER_RADIAN_PER_UNIT,
                SHOULDER_MAX_ACC_RADIAN_PER_SEC_SQ / SHOULDER_RADIAN_PER_UNIT),
                (SHOULDER_OFFSET_RADIAN-dutyCycleEncoder.getDistance() * 2 * Math.PI) / SHOULDER_RADIAN_PER_UNIT);

        m_motorLeader = new WPI_TalonFX(Constants.SHOOTER_ONE_ID);
        m_motorFollower = new WPI_TalonFX(Constants.SHOOTER_TWO_ID);

        m_motorLeader.configFactoryDefault();
        m_motorFollower.configFactoryDefault();

        m_shooterEncoder = m_motorLeader.getSensorCollection();

        m_motorLeader.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
        // Set follower
        m_motorFollower.follow(m_motorLeader, FollowerType.PercentOutput);

        m_motorLeader.config_kF(kPIDLoopIdx, SHOULDER_K_FF, kTimeoutMs);
		m_motorLeader.config_kP(kPIDLoopIdx, SHOULDER_K_P, kTimeoutMs);
		m_motorLeader.config_kI(kPIDLoopIdx, SHOULDER_K_I, kTimeoutMs);
		m_motorLeader.config_kD(kPIDLoopIdx, SHOULDER_K_D, kTimeoutMs);

        // limits for motor leader and folower
        // always limit current to the values. Trigger limit = 0 so that it is always enforced.
        m_motorLeader.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, LEADER_CURRENT_LIMIT, 0, 0));
        m_motorFollower.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, FOLLOW_CURRENT_LIMIT, 0, 0));

        m_dutyEncoder = dutyCycleEncoder;

        // Encoder distance is in radians
        m_dutyEncoder.setDistancePerRotation(2 * Math.PI);
        m_dutyEncoder.setPositionOffset(SHOULDER_POSITION_OFFSET);

        double initialAngle = -m_dutyEncoder.getDistance();
        // SmartDashboard.putNumber("shoulder/initAngle", Units.radiansToDegrees(initialAngle));

        // Set the motor encoder and Position setpoint to the initialAngle from the absolute encoder
        m_shooterEncoder.setIntegratedSensorPosition(initialAngle / SHOULDER_RADIAN_PER_UNIT, 0);

        SmartDashboard.putNumber("shoulder/leaderCurrentLimit", LEADER_CURRENT_LIMIT);
        SmartDashboard.putNumber("shoulder/followCurrentLimit", FOLLOW_CURRENT_LIMIT);
        // m_motorLeader.setSelectedSensorPosition(-m_Duty_Encoder.getDistance());
        // m_motorLeader.setSelectedSensorPosition(m_encoder.getIntegratedSensorPosition());
        // m_motorLeader.setSelectedSensorPosition(initialAngle / SHOULDER_RADIAN_PER_UNIT);
        // SmartDashboard.putNumber("shoulder/motorLeaderIntegSensPos", m_motorLeader.getSelectedSensorPosition());

        // m_Duty_Encoder.setPositionOffset(SHOULDER_OFFSET_RADIAN);
        // m_motorSim = new TalonFXSimCollection(m_motorLeader);
        // m_encoderSim = new TalonFXSimCollection(m_encoder);
        // SmartDashboard.putNumber("shoulder/absoluteEncoder", Math.toDegrees(-m_dutyEncoder.getDistance()));
        // SmartDashboard.putNumber("shoulder/P Gain", m_kPShoulder);
        // SmartDashboard.putData("shoulder Sim", m_mech2d);

        setCoastMode(false);
        SmartDashboard.putBoolean("shoulder/coastMode", m_coastMode);
        // SmartDashboard.putNumber("shoulder/kFeedForward", m_kFeedForward);

        // m_motorLeader.set(ControlMode.Position, )
        // m_motorLeader.set(ControlMode.Position, m_encoder.getIntegratedSensorPosition(), DemandType.ArbitraryFeedForward, 0.0);//feedforward/12.0);
        // setAngle(m_encoder.getIntegratedSensorPosition() * SHOULDER_RADIAN_PER_UNIT);

    // We want motor2 to be master and motor1 and 3 follow the speed of motor2
    m_motorLeader.follow(m_motorFollower);
    // motor3.follow(motor2);

    m_motorLeader.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 0, 0));
    m_motorFollower.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 0, 0));
    // motor3.setSmartCurrentLimit(40);
    
    // Reset Smart Dashboard for shooter test
    SmartDashboard.putString("shooter/Status", "Idle");

    // always have an entry at 0 so that it has a chance of working at short distances
    distanceLookUp.put(0.0, new Double[] { 5500.0, 103.0 });
    distanceLookUp.put(50.0, new Double[] { 5500.0, 103.0 });
    distanceLookUp.put(110.0, new Double[] { 6500.0, 87.0 });
    distanceLookUp.put(170.0, new Double[] { 8500.0, 70.0 });
    distanceLookUp.put(230.0, new Double[] { 9000.0, 60.0 });
    distanceLookUp.put(318.1, new Double[] { 9000.0, 55.0 });
    // extra far, to make sure the table work at the long end
    distanceLookUp.put(400.0, new Double[] { 9000.0, 50.0 });

    // The relative setting for non-zero angles needs to be recomputed if the zero setting chenges,
    // but at least we'll be close
    turretAngleLookup.put(-5.0, Constants.TURRET_ANGLE_ZERO_SETTING - 23.0);
    turretAngleLookup.put(-4.0, Constants.TURRET_ANGLE_ZERO_SETTING - 18.0);
    turretAngleLookup.put(-3.0, Constants.TURRET_ANGLE_ZERO_SETTING - 11.0);
    turretAngleLookup.put(-2.0, Constants.TURRET_ANGLE_ZERO_SETTING - 6.0);
    turretAngleLookup.put(-1.0, Constants.TURRET_ANGLE_ZERO_SETTING - 4.0);
    turretAngleLookup.put( 0.0, Constants.TURRET_ANGLE_ZERO_SETTING);
    turretAngleLookup.put( 1.0, Constants.TURRET_ANGLE_ZERO_SETTING + 4.0);
    turretAngleLookup.put( 2.0, Constants.TURRET_ANGLE_ZERO_SETTING + 7.0);
    turretAngleLookup.put( 3.0, Constants.TURRET_ANGLE_ZERO_SETTING + 10.5);
    turretAngleLookup.put( 4.0, Constants.TURRET_ANGLE_ZERO_SETTING + 13.0);
    turretAngleLookup.put( 5.0, Constants.TURRET_ANGLE_ZERO_SETTING + 15.0);

    // used in ShooterPIDTuner
    SmartDashboard.putNumber("shooter/P", 0.000145);
    SmartDashboard.putNumber("shooter/I", 1e-8);
    SmartDashboard.putNumber("shooter/D", 0);
    SmartDashboard.putNumber("shooter/F", 6.6774 * 0.00001);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("shooter/RPM", getSpeed());
    SmartDashboard.putNumber("shooter/current", m_motorFollower.getSupplyCurrent());
    SmartDashboard.putNumber("shooter/distance", m_vision.getDistance());
    SmartDashboard.putNumber("shooter/Hood_Adjustment", m_hoodAdjustment);
    SmartDashboard.putNumber("shooter/RPM_Adjustment", m_rpmAdjustment);
    SmartDashboard.putNumber("shooter/Output_Voltage", m_motorFollower.getMotorOutputVoltage());
    SmartDashboard.putNumber("shooter/turretAngleRaw", turretServo.getAngle());
  }

  public void setCoastMode(boolean coastMode){
        if (coastMode) {
            m_motorLeader.setNeutralMode(NeutralMode.Coast);
            m_motorLeader.stopMotor();
        } else
            m_motorLeader.setNeutralMode(NeutralMode.Brake);
    }

  public double getVoltage() {
    return m_motorFollower.getBusVoltage();
  }

  public void setHood(double angle) {
    System.out.println("hood angle SET!!!!");
    if (angle < 40) {
        angle = 40;
    }
    if (angle > 160) {
        angle = 160;
    }
    hoodServo.setAngle(angle);
  }

  public double getSpeed() {
    return -m_shooterEncoder.getIntegratedSensorVelocity();
  }

  public void prepareShooter(double distance) {
    // Set the shooter and hood based on the distance
    setShooterRpm(calculateShooterSpeed(distance));
    setHood(calculateShooterHood(distance));
  }

  // public void setShooterVoltage (double voltage) {
  //     pidController.setReference(voltage, ControlType.kVoltage);
  // }

  public void shoot() {
    //if (flup.getOutputCurrent() < Constants.FLUP_STOP_CURRENT) {
        flup.set(-0.5);
    //} else {
    //    flup.set(0);
    //}*/
  }

  public void testSpin() {
    setShooterRpm(4000.0);
    SmartDashboard.putString("shooter/Status", "Shooting");
  }

  public void setShooterRpm(double rpm) {
    System.out.println("Shooter RPM SET!!!!!");
    // for the shooter to run the right direction, rpm values passed to setReference must be negative
    // passing the negative absolute value causes the passed value to always be negative, 
    // while allowing the function argument to be positive or negative  
    if (rpm < 0) System.out.println("warning: shooter rpm argument should be positive");
    // pidController.setReference(-Math.abs(rpm), ControlType.kVelocity, 0, -0.8);
  }

  public double calculateShooterSpeed(double distance) {
    Entry<Double, Double[]> floorEntry = distanceLookUp.floorEntry(distance);
    Entry<Double, Double[]> ceilingEntry = distanceLookUp.higherEntry(distance);
    if (floorEntry != null && ceilingEntry != null) {
      // Charles' calculation
      double ratio = 1 - (ceilingEntry.getKey() - distance) / (ceilingEntry.getKey() - floorEntry.getKey());
      double rpmAdjustment = floorEntry.getValue()[0] + ratio * (ceilingEntry.getValue()[0] - floorEntry.getValue()[0]);

      System.out.format("Shooter: ratio %3.2f, floor %4.1f, dist %4.1f, ceiling %4.1f, RPM %4.1f",
      ratio, floorEntry.getKey(), distance,  ceilingEntry.getKey(), rpmAdjustment);
      return rpmAdjustment;
    }
    else {
      System.out.println("Shooter: floorEntry or ceilingEntry was null");
      // Typical speed. Not sure this will work for much, but it won't break anything.
      return 4000;
    }
  }

  public double calculateShooterHood(double distance) {
    Entry<Double, Double[]> floorEntry = distanceLookUp.floorEntry(distance);
    Entry<Double, Double[]> ceilingEntry = distanceLookUp.higherEntry(distance);

    if (floorEntry != null && ceilingEntry != null) {
      // Charles calculation
      double ratio = 1 - (ceilingEntry.getKey() - distance) / (ceilingEntry.getKey() - floorEntry.getKey());
      double hoodAdjustment = floorEntry.getValue()[1] + ratio * (ceilingEntry.getValue()[1] - floorEntry.getValue()[1]);
      System.out.format(" hood %3.0f%n", hoodAdjustment);

      return hoodAdjustment;
    }
    else {
      return 60;
    }
  }

  public void warmUp() {
    setShooterRpm(Constants.WARM_UP_RPM);
  }

  public boolean speedOnTarget(final double targetVelocity, final double percentAllowedError) {
    final double max = targetVelocity * (1.0 + (percentAllowedError / 100.0));
    final double min = targetVelocity * (1.0 - (percentAllowedError / 100.0));
    return m_shooterEncoder.getIntegratedSensorVelocity() > max && m_shooterEncoder.getIntegratedSensorVelocity() < min;  //this is wack cause it's negative
  }

  public boolean hoodOnTarget(final double targetAngle) {
    System.out.println("ServoPosition: " + hoodServo.getPosition());
    return hoodServo.getAngle() > targetAngle - 0.5 && hoodServo.getAngle() < targetAngle + 0.5;
  }

  /*
  public void calibratePID(final double p, final double i, final double d, final double f) {
    pidController.setIAccum(0);
    pidController.setP(p);
    pidController.setI(i);
    pidController.setD(d);
    pidController.setFF(f);
    pidController.setIZone(1000);
  }
  */

  public void stopAll() {
    setShooterRpm(0.0);
    flup.set(0);
    setHood(160);
    SmartDashboard.putString("shooter/Status", "Idle");
  }

  public double getTurretAngle() {
    return turretServo.get() *  Constants.TURRET_ANGLE_COEFFICIENT;
  }

  private void setTurret(double angle) {
    System.out.println("Moving turret to " + angle);
    turretServo.setAngle(angle);
  }

  public void setTurretAdjusted(double adjustedAngle) {
    SmartDashboard.putNumber("shooter/turretAngle", adjustedAngle);
    if (adjustedAngle > 5) {
      adjustedAngle = 5;
    }
    if (adjustedAngle < -5) {
      adjustedAngle = -5;
    }
    Entry<Double, Double> floorEntry = adjustedAngle < 0 ? turretAngleLookup.higherEntry(adjustedAngle) : 
                                                            turretAngleLookup.floorEntry(adjustedAngle);
    Entry<Double, Double> ceilingEntry = adjustedAngle < 0 ? turretAngleLookup.floorEntry(adjustedAngle) :  
                                                            turretAngleLookup.higherEntry(adjustedAngle);
                                                    
    if (floorEntry != null && ceilingEntry != null) {
      // Charles calculation
      double ratio = 1 - (ceilingEntry.getKey() - adjustedAngle) / (ceilingEntry.getKey() - floorEntry.getKey());
      double result = floorEntry.getValue() + ratio * (ceilingEntry.getValue() - floorEntry.getValue());

      setTurret(result);
      // System.out.println("Turret Adjustment should be working: " + result + "    " + adjustedAngle);
    }
    else {
      System.out.println("Turret Adjustment not successful      " + adjustedAngle);
      // This must equal the zero setting in turretAngleLookup
      setTurret(Constants.TURRET_ANGLE_ZERO_SETTING);
    }
  }

  @Override
  protected void useState(State state) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'useState'");
  }
}
