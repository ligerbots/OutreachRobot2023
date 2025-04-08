// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map.Entry;
import java.util.TreeMap;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    TalonFX m_shooter;
    // the thing that actually does the shooting
    TalonFX m_flup;

    //TODO: Replace with motor
    // Servo m_hoodServo;

    TreeMap<Double, Double[]> m_distanceLookUp = new TreeMap<Double, Double[]>() {
    }; // set up lookup table for ranges
    TreeMap<Double, Double> m_turretAngleLookup = new TreeMap<Double, Double>() {
    };
    DutyCycleEncoder m_dutyEncoder;

    private static final double SHOOTER_STATOR_CURRENT_LIMIT = 40.0;

    // PID Constants for the shooter PID controller
    private static final double SHOOTER_K_P = 0.12;

    // constructor
    public Shooter() {
        m_shooter = new TalonFX(Constants.SHOOTER_CAN_ID);
        m_shooter.getConfigurator().apply(new TalonFXConfiguration());

        // m_shooter.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs)

        // PID Configs
        var talonFXConfigs = new TalonFXConfiguration();
        // set slot 0 gains and leave every other config factory-default
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kP = SHOOTER_K_P;

        m_shooter.getConfigurator().apply(talonFXConfigs, 0.1);

        // Supply current limiting
        CurrentLimitsConfigs shooterCurrentLimitConfigs = new CurrentLimitsConfigs();
        shooterCurrentLimitConfigs.StatorCurrentLimit = SHOOTER_STATOR_CURRENT_LIMIT;
        shooterCurrentLimitConfigs.StatorCurrentLimitEnable = true;

        m_shooter.getConfigurator().apply(shooterCurrentLimitConfigs);

        // // always have an entry at 0 so that it has a chance of working at short
        // // distances
        // m_distanceLookUp.put(0.0, new Double[] { 5500.0, 103.0 });
        // m_distanceLookUp.put(50.0, new Double[] { 5500.0, 103.0 });
        // m_distanceLookUp.put(110.0, new Double[] { 6500.0, 87.0 });
        // m_distanceLookUp.put(170.0, new Double[] { 8500.0, 70.0 });
        // m_distanceLookUp.put(230.0, new Double[] { 9000.0, 60.0 });
        // m_distanceLookUp.put(318.1, new Double[] { 9000.0, 55.0 });
        // // extra far, to make sure the table work at the long end
        // m_distanceLookUp.put(400.0, new Double[] { 9000.0, 50.0 });

        // // The relative setting for non-zero angles needs to be recomputed if the zero
        // // setting chenges,
        // // but at least we'll be close
        // m_turretAngleLookup.put(-5.0, Constants.TURRET_ANGLE_ZERO_SETTING - 23.0);
        // m_turretAngleLookup.put(-4.0, Constants.TURRET_ANGLE_ZERO_SETTING - 18.0);
        // m_turretAngleLookup.put(-3.0, Constants.TURRET_ANGLE_ZERO_SETTING - 11.0);
        // m_turretAngleLookup.put(-2.0, Constants.TURRET_ANGLE_ZERO_SETTING - 6.0);
        // m_turretAngleLookup.put(-1.0, Constants.TURRET_ANGLE_ZERO_SETTING - 4.0);
        // m_turretAngleLookup.put(0.0, Constants.TURRET_ANGLE_ZERO_SETTING);
        // m_turretAngleLookup.put(1.0, Constants.TURRET_ANGLE_ZERO_SETTING + 4.0);
        // m_turretAngleLookup.put(2.0, Constants.TURRET_ANGLE_ZERO_SETTING + 7.0);
        // m_turretAngleLookup.put(3.0, Constants.TURRET_ANGLE_ZERO_SETTING + 10.5);
        // m_turretAngleLookup.put(4.0, Constants.TURRET_ANGLE_ZERO_SETTING + 13.0);
        // m_turretAngleLookup.put(5.0, Constants.TURRET_ANGLE_ZERO_SETTING + 15.0);
    }

    // called a lot
    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooter/RPM", getSpeed());
        SmartDashboard.putNumber("shooter/current", m_shooter.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("shooter/Output_Voltage", m_shooter.getMotorVoltage().getValueAsDouble());
    }

    public double getVoltage() {
        return m_shooter.getSupplyVoltage().getValueAsDouble();
    }

    // public void setHood(double angle) {
    //     System.out.println("hood angle SET!!!!");
    //     if (angle < 40) {
    //         angle = 40;
    //     }
    //     if (angle > 160) {
    //         angle = 160;
    //     }
    //     m_hoodServo.setAngle(angle);
    // }

    public double getSpeed() {
        //return -m_shooterEncoder.getIntegratedSensorVelocity();
        return -m_shooter.getVelocity().getValueAsDouble();
    }

    public void prepareShooter(double distance) {
        // Set the shooter and hood based on the distance
        setShooterRpm(calculateShooterSpeed(distance));
        // setHood(calculateShooterHood(distance));
    }

    // SHOOT!!!!!!!!!!
    public void shoot() {
        m_flup.set(-0.5);
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

    public boolean speedOnTarget(final double targetVelocity, final double percentAllowedError) {
        final double max = targetVelocity * (1.0 + (percentAllowedError / 100.0));
        final double min = targetVelocity * (1.0 - (percentAllowedError / 100.0));
        return m_shooter.getVelocity().getValueAsDouble() > max
                && m_shooter.getVelocity().getValueAsDouble() < min; // this is wack cause it's negative
    }

    // public boolean hoodOnTarget(final double targetAngle) {
    //     System.out.println("ServoPosition: " + m_hoodServo.getPosition());
    //     return m_hoodServo.getAngle() > targetAngle - 0.5 && m_hoodServo.getAngle() < targetAngle + 0.5;
    // }

    public void stopAll() {
        setShooterRpm(0.0);
        m_flup.set(0);
        // setHood(160);
        SmartDashboard.putString("shooter/Status", "Idle");
    }
}
