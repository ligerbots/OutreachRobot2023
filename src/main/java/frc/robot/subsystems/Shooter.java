// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map.Entry;
import java.util.TreeMap;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class Shooter extends TrapezoidProfileSubsystem {
    TalonFX m_motorLeader, m_motorFollower;
    // the thing that actually does the shooting
    TalonFX m_flup;
    TalonFXSensorCollection m_shooterEncoder;
    Servo m_hoodServo, m_turretServo;
    TreeMap<Double, Double[]> m_distanceLookUp = new TreeMap<Double, Double[]>() {
    }; // set up lookup table for ranges
    TreeMap<Double, Double> m_turretAngleLookup = new TreeMap<Double, Double>() {
    };
    DutyCycleEncoder m_dutyEncoder;

    public int m_rpmAdjustment = 0;
    public int m_hoodAdjustment = 0;
    public double m_angleErrorAfterTurn = 0;

    // private static final double SHOULDER_MAX_ANGLE = Math.toRadians(30.0);
    // private static final double SHOULDER_MIN_ANGLE = Math.toRadians(-65.0);

    public static final double SHOULDER_ANGLE_TOLERANCE_RADIAN = Math.toRadians(3.0);

    private static final double LEADER_CURRENT_LIMIT = 40.0;
    private static final double FOLLOW_CURRENT_LIMIT = 40.0;

    // TODO: The following constants came from the 2022 robot.
    // These need to be set for this robot.

    // All units are MKS with angles in Radians

    // Feedforward constants for the shoulder
    // private static final double SHOULDER_KS = 0.182; // TODO: This may need to be
    // tuned
    // The following constants are computed from https://www.reca.lc/arm
    // private static final double SHOULDER_KG = 0.09; // V
    // private static final double SHOULDER_KV = 6.60; // V*sec/rad
    // private static final double SHOULDER_KA = 0.01; // V*sec^2/rad

    // Constants to limit the shoulder rotation speed
    private static final double SHOULDER_MAX_VEL_RADIAN_PER_SEC = Units.degreesToRadians(300.0); // 120 deg/sec
    private static final double SHOULDER_MAX_ACC_RADIAN_PER_SEC_SQ = Units.degreesToRadians(450.0); // 120 deg/sec^2

    private static final double SHOULDER_POSITION_OFFSET = 62.0 / 360.0;
    private static final double SHOULDER_OFFSET_RADIAN = SHOULDER_POSITION_OFFSET * 2 * Math.PI;

    // The Shoulder gear ratio is 288, but let's get it exactly.
    // private static final double SHOULDER_GEAR_RATIO = (84.0 /12.0) * (84.0 /
    // 18.0) * (84.0 / 26.0) * (60.0 / 22.0);
    private static final double SHOULDER_GEAR_RATIO = (84.0 / 12.0) * (84.0 / 18.0) * (70.0 / 40.0) * (60.0 / 22.0);

    // PID Constants for the shoulder PID controller
    // Since we're using Trapeziodal control, all values will be 0 except for P
    private static final double SHOULDER_K_P = 0.15;
    private static final double SHOULDER_K_I = 0;
    private static final double SHOULDER_K_D = 0;
    private static final double SHOULDER_K_FF = 0;
    private static final int kPIDLoopIdx = 0;
    private static final int kTimeoutMs = 0;
    private boolean m_coastMode = false;

    // The TalonFX, the integrated motor controller for the Falcon, uses ticks as
    // it's noative unit.
    // There are 2048 ticks per revolution. Need to account for the gear ratio.
    private static final double SHOULDER_RADIAN_PER_UNIT = 2 * Math.PI / (2048 * SHOULDER_GEAR_RATIO);

    // constructor
    public Shooter(DutyCycleEncoder dutyCycleEncoder) {
        super(new TrapezoidProfile.Constraints(SHOULDER_MAX_VEL_RADIAN_PER_SEC / SHOULDER_RADIAN_PER_UNIT,
                SHOULDER_MAX_ACC_RADIAN_PER_SEC_SQ / SHOULDER_RADIAN_PER_UNIT),
                (SHOULDER_OFFSET_RADIAN - dutyCycleEncoder.getDistance() * 2 * Math.PI) / SHOULDER_RADIAN_PER_UNIT);

        m_motorLeader = new TalonFX(Constants.SHOOTER_ONE_ID);
        m_motorFollower = new TalonFX(Constants.SHOOTER_TWO_ID);

        m_motorLeader.getConfigurator().apply(new TalonFXConfiguration());
        m_motorFollower.getConfigurator().apply(new TalonFXConfiguration());

        m_shooterEncoder = m_motorLeader.getSensorCollection();

        m_motorLeader.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
        // Set follower
        m_motorFollower.follow(m_motorLeader, FollowerType.PercentOutput);

        var talonFXConfigs = new TalonFXConfiguration();
        // set slot 0 gains and leave every other config factory-default
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kV = kTimeoutMs;
        slot0Configs.kP = SHOULDER_K_P;
        slot0Configs.kI = SHOULDER_K_I;
        slot0Configs.kD = SHOULDER_K_D;

        // apply all configs
        m_motorLeader.getConfigurator().apply(talonFXConfigs, kTimeoutMs);

        // limits for motor leader and folower
        // always limit current to the values. Trigger limit = 0 so that it is always
        // enforced.
        m_motorLeader.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, LEADER_CURRENT_LIMIT, 0, 0));
        m_motorFollower.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, FOLLOW_CURRENT_LIMIT, 0, 0));

        m_dutyEncoder = dutyCycleEncoder;

        // Encoder distance is in radians
        m_dutyEncoder.setDistancePerRotation(2 * Math.PI);
        m_dutyEncoder.setPositionOffset(SHOULDER_POSITION_OFFSET);

        double initialAngle = -m_dutyEncoder.getDistance();
        // SmartDashboard.putNumber("shoulder/initAngle",
        // Units.radiansToDegrees(initialAngle));

        // Set the motor encoder and Position setpoint to the initialAngle from the
        // absolute encoder
        m_shooterEncoder.setIntegratedSensorPosition(initialAngle / SHOULDER_RADIAN_PER_UNIT, 0);

        SmartDashboard.putNumber("shoulder/leaderCurrentLimit", LEADER_CURRENT_LIMIT);
        SmartDashboard.putNumber("shoulder/followCurrentLimit", FOLLOW_CURRENT_LIMIT);
        // m_motorLeader.setSelectedSensorPosition(-m_Duty_Encoder.getDistance());
        // m_motorLeader.setSelectedSensorPosition(m_encoder.getIntegratedSensorPosition());
        // m_motorLeader.setSelectedSensorPosition(initialAngle /
        // SHOULDER_RADIAN_PER_UNIT);
        // SmartDashboard.putNumber("shoulder/motorLeaderIntegSensPos",
        // m_motorLeader.getSelectedSensorPosition());

        // m_Duty_Encoder.setPositionOffset(SHOULDER_OFFSET_RADIAN);
        // m_motorSim = new TalonFXSimCollection(m_motorLeader);
        // m_encoderSim = new TalonFXSimCollection(m_encoder);
        // SmartDashboard.putNumber("shoulder/absoluteEncoder",
        // Math.toDegrees(-m_dutyEncoder.getDistance()));
        // SmartDashboard.putNumber("shoulder/P Gain", m_kPShoulder);
        // SmartDashboard.putData("shoulder Sim", m_mech2d);

        setCoastMode(false);
        SmartDashboard.putBoolean("shoulder/coastMode", m_coastMode);
        // SmartDashboard.putNumber("shoulder/kFeedForward", m_kFeedForward);

        // m_motorLeader.set(ControlMode.Position, )
        // m_motorLeader.set(ControlMode.Position,
        // m_encoder.getIntegratedSensorPosition(), DemandType.ArbitraryFeedForward,
        // 0.0);//feedforward/12.0);
        // setAngle(m_encoder.getIntegratedSensorPosition() * SHOULDER_RADIAN_PER_UNIT);

        // We want motor2 to be master and motor1 and 3 follow the speed of motor2
        m_motorLeader.follow(m_motorFollower);
        // motor3.follow(motor2);

        m_motorLeader.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 0, 0));
        m_motorFollower.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 0, 0));
        // motor3.setSmartCurrentLimit(40);

        // setup PID control for TalonFX
        m_motorLeader.configFactoryDefault();
        m_motorLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        m_motorLeader.set(ControlMode.Position, 0);
        m_motorLeader.config_kP(0, 50);
        m_motorLeader.config_kI(0, 0);
        m_motorLeader.config_kD(0, 0);
        m_motorLeader.config_kF(0, 0);
        m_motorLeader.setSensorPhase(false);

        setMotorMode(NeutralMode.Coast);

        // m_leftEncoder.setDistancePerPulse(Constants.ENCODER_DISTANCE_PER_PULSE);
        // m_rightEncoder.setDistancePerPulse(Constants.ENCODER_DISTANCE_PER_PULSE);
        // m_rightEncoder.setReverseDirection(true);

        // if (RobotBase.isSimulation()) {

        // m_differentialDriveSim = new DifferentialDrivetrainSim(
        // Constants.kDrivetrainPlant,
        // Constants.kDriveGearbox,
        // Constants.kDriveGearing,
        // Constants.kTrackwidth,
        // Constants.kWheelDiameterMeters / 2.0,
        // null);

        // m_leftLeader_sim = m_leftLeader.getSimCollection();
        // m_rightLeader_sim = m_rightLeader.getSimCollection();
        // // m_leftEncoderSim = new EncoderSim(m_leftEncoder);
        // // m_rightEncoderSim = new EncoderSim(m_rightEncoder);
        // m_gyroAngleSim = new SimDeviceSim("navX-Sensor[0]").getDouble("Yaw");

        // m_fieldSim = new Field2d();
        // SmartDashboard.putData("Field", m_fieldSim);
        // }

        // Reset Smart Dashboard for shooter test
        SmartDashboard.putString("shooter/Status", "Idle");

        // always have an entry at 0 so that it has a chance of working at short
        // distances
        m_distanceLookUp.put(0.0, new Double[] { 5500.0, 103.0 });
        m_distanceLookUp.put(50.0, new Double[] { 5500.0, 103.0 });
        m_distanceLookUp.put(110.0, new Double[] { 6500.0, 87.0 });
        m_distanceLookUp.put(170.0, new Double[] { 8500.0, 70.0 });
        m_distanceLookUp.put(230.0, new Double[] { 9000.0, 60.0 });
        m_distanceLookUp.put(318.1, new Double[] { 9000.0, 55.0 });
        // extra far, to make sure the table work at the long end
        m_distanceLookUp.put(400.0, new Double[] { 9000.0, 50.0 });

        // The relative setting for non-zero angles needs to be recomputed if the zero
        // setting chenges,
        // but at least we'll be close
        m_turretAngleLookup.put(-5.0, Constants.TURRET_ANGLE_ZERO_SETTING - 23.0);
        m_turretAngleLookup.put(-4.0, Constants.TURRET_ANGLE_ZERO_SETTING - 18.0);
        m_turretAngleLookup.put(-3.0, Constants.TURRET_ANGLE_ZERO_SETTING - 11.0);
        m_turretAngleLookup.put(-2.0, Constants.TURRET_ANGLE_ZERO_SETTING - 6.0);
        m_turretAngleLookup.put(-1.0, Constants.TURRET_ANGLE_ZERO_SETTING - 4.0);
        m_turretAngleLookup.put(0.0, Constants.TURRET_ANGLE_ZERO_SETTING);
        m_turretAngleLookup.put(1.0, Constants.TURRET_ANGLE_ZERO_SETTING + 4.0);
        m_turretAngleLookup.put(2.0, Constants.TURRET_ANGLE_ZERO_SETTING + 7.0);
        m_turretAngleLookup.put(3.0, Constants.TURRET_ANGLE_ZERO_SETTING + 10.5);
        m_turretAngleLookup.put(4.0, Constants.TURRET_ANGLE_ZERO_SETTING + 13.0);
        m_turretAngleLookup.put(5.0, Constants.TURRET_ANGLE_ZERO_SETTING + 15.0);

        // used in ShooterPIDTuner
        SmartDashboard.putNumber("shooter/P", 0.000145);
        SmartDashboard.putNumber("shooter/I", 1e-8);
        SmartDashboard.putNumber("shooter/D", 0);
        SmartDashboard.putNumber("shooter/F", 6.6774 * 0.00001);
    }

    private void setMotorMode(NeutralMode coast) {
    }

    // called a lot
    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooter/RPM", getSpeed());
        SmartDashboard.putNumber("shooter/current", m_motorFollower.getSupplyCurrent());
        SmartDashboard.putNumber("shooter/distance", m_vision.getDistance());
        SmartDashboard.putNumber("shooter/Hood_Adjustment", m_hoodAdjustment);
        SmartDashboard.putNumber("shooter/RPM_Adjustment", m_rpmAdjustment);
        SmartDashboard.putNumber("shooter/Output_Voltage", m_motorFollower.getMotorOutputVoltage());
        SmartDashboard.putNumber("shooter/turretAngleRaw", m_turretServo.getAngle());
    }

    public void setCoastMode(boolean coastMode) {
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
        m_hoodServo.setAngle(angle);
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
    // pidController.setReference(voltage, ControlType.kVoltage);
    // }

    // SHOOT!!!!!!!!!!
    public void shoot() {
        // if (flup.getOutputCurrent() < Constants.FLUP_STOP_CURRENT) {
        m_flup.set(-0.5);
        // } else {
        // flup.set(0);
        // }*/
    }

    public void testSpin() {
        setShooterRpm(4000.0);
        SmartDashboard.putString("shooter/Status", "Shooting");
    }

    // does literally nothing
    public void setShooterRpm(double rpm) {
        System.out.println("Shooter RPM SET!!!!!");
        // for the shooter to run the right direction, rpm values passed to setReference
        // must be negative
        // passing the negative absolute value causes the passed value to always be
        // negative,
        // while allowing the function argument to be positive or negative
        if (rpm < 0)
            System.out.println("warning: shooter rpm argument should be positive");
        // pidController.setReference(-Math.abs(rpm), ControlType.kVelocity, 0, -0.8);
    }

    public double calculateShooterSpeed(double distance) {
        Entry<Double, Double[]> floorEntry = m_distanceLookUp.floorEntry(distance);
        Entry<Double, Double[]> ceilingEntry = m_distanceLookUp.higherEntry(distance);
        if (floorEntry != null && ceilingEntry != null) {
            // Charles' calculation
            double ratio = 1 - (ceilingEntry.getKey() - distance) / (ceilingEntry.getKey() - floorEntry.getKey());
            double rpmAdjustment = floorEntry.getValue()[0]
                    + ratio * (ceilingEntry.getValue()[0] - floorEntry.getValue()[0]);

            System.out.format("Shooter: ratio %3.2f, floor %4.1f, dist %4.1f, ceiling %4.1f, RPM %4.1f",
                    ratio, floorEntry.getKey(), distance, ceilingEntry.getKey(), rpmAdjustment);
            return rpmAdjustment;
        } else {
            System.out.println("Shooter: floorEntry or ceilingEntry was null");
            // Typical speed. Not sure this will work for much, but it won't break anything.
            return 4000;
        }
    }

    public double calculateShooterHood(double distance) {
        Entry<Double, Double[]> floorEntry = m_distanceLookUp.floorEntry(distance);
        Entry<Double, Double[]> ceilingEntry = m_distanceLookUp.higherEntry(distance);

        if (floorEntry != null && ceilingEntry != null) {
            // Charles calculation
            double ratio = 1 - (ceilingEntry.getKey() - distance) / (ceilingEntry.getKey() - floorEntry.getKey());
            double hoodAdjustment = floorEntry.getValue()[1]
                    + ratio * (ceilingEntry.getValue()[1] - floorEntry.getValue()[1]);
            System.out.format(" hood %3.0f%n", hoodAdjustment);

            return hoodAdjustment;
        } else {
            return 60;
        }
    }

    public void warmUp() {
        setShooterRpm(Constants.WARM_UP_RPM);
    }

    public boolean speedOnTarget(final double targetVelocity, final double percentAllowedError) {
        final double max = targetVelocity * (1.0 + (percentAllowedError / 100.0));
        final double min = targetVelocity * (1.0 - (percentAllowedError / 100.0));
        return m_shooterEncoder.getIntegratedSensorVelocity() > max
                && m_shooterEncoder.getIntegratedSensorVelocity() < min; // this is wack cause it's negative
    }

    public boolean hoodOnTarget(final double targetAngle) {
        System.out.println("ServoPosition: " + m_hoodServo.getPosition());
        return m_hoodServo.getAngle() > targetAngle - 0.5 && m_hoodServo.getAngle() < targetAngle + 0.5;
    }

    public void calibratePID(final double p, final double i, final double d, final double f) {
        m_motorLeader.config_kP(0, p);
        m_motorLeader.config_kI(0, i);
        m_motorLeader.config_kD(0, d);
        m_motorLeader.config_kF(0, f);
    }

    public void stopAll() {
        setShooterRpm(0.0);
        m_flup.set(0);
        setHood(160);
        SmartDashboard.putString("shooter/Status", "Idle");
    }

    public double getTurretAngle() {
        return m_turretServo.get() * Constants.TURRET_ANGLE_COEFFICIENT;
    }

    private void setTurret(double angle) {
        System.out.println("Moving turret to " + angle);
        m_turretServo.setAngle(angle);
    }

    public void setTurretAdjusted(double adjustedAngle) {
        SmartDashboard.putNumber("shooter/turretAngle", adjustedAngle);
        if (adjustedAngle > 5) {
            adjustedAngle = 5;
        }
        if (adjustedAngle < -5) {
            adjustedAngle = -5;
        }
        Entry<Double, Double> floorEntry = adjustedAngle < 0 ? m_turretAngleLookup.higherEntry(adjustedAngle)
                : m_turretAngleLookup.floorEntry(adjustedAngle);
        Entry<Double, Double> ceilingEntry = adjustedAngle < 0 ? m_turretAngleLookup.floorEntry(adjustedAngle)
                : m_turretAngleLookup.higherEntry(adjustedAngle);

        if (floorEntry != null && ceilingEntry != null) {
            // Charles calculation
            double ratio = 1 - (ceilingEntry.getKey() - adjustedAngle) / (ceilingEntry.getKey() - floorEntry.getKey());
            double result = floorEntry.getValue() + ratio * (ceilingEntry.getValue() - floorEntry.getValue());

            setTurret(result);
            // System.out.println("Turret Adjustment should be working: " + result + " " +
            // adjustedAngle);
        } else {
            System.out.println("Turret Adjustment not successful      " + adjustedAngle);
            // This must equal the zero setting in turretAngleLookup
            setTurret(Constants.TURRET_ANGLE_ZERO_SETTING);
        }
    }

    @Override
    protected void useState(State state) {
        // TODO Auto-generated method stub
        // throw new UnsupportedOperationException("Unimplemented method 'useState'");
    }

}
