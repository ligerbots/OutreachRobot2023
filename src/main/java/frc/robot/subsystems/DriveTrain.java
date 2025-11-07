// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;

import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
    
    private SparkMax m_leftMotor = new SparkMax(Constants.LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
    private SparkMax m_rightMotor = new SparkMax(Constants.RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);
    
    private DifferentialDrive m_robotDrive;
    private DifferentialDriveOdometry m_odometry;
    
    private RelativeEncoder m_leftEncoder; 
    private RelativeEncoder m_rightEncoder; 
    
    private final AHRS m_navX = new AHRS(NavXComType.kMXP_SPI);
    
    private static final int CURRENT_LIMIT = 35;
    
    private static final double MAX_DRIVE_SPEED = 0.66;
    private static final double MAX_TURN_SPEED = 0.66;

    private static final double DRIVE_GEAR_REDUCTION = 1/19.527; //TODO: Double check number
    private static final double METER_PER_REVOLUTION = (Units.inchesToMeters(8)*Math.PI) * DRIVE_GEAR_REDUCTION;
    
    public DriveTrain() {
        m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
        
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        // always set a current limit
        config.smartCurrentLimit(CURRENT_LIMIT);
        config.idleMode(IdleMode.kBrake);

        config.encoder.positionConversionFactor(METER_PER_REVOLUTION);

        m_leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        config.inverted(false);
        m_rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        m_leftEncoder = m_leftMotor.getEncoder();
        m_rightEncoder = m_rightMotor.getEncoder();
        
        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0), 0, 0);
    }
    
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }
    
    public void setPose(Pose2d pose) {
        // The left and right encoders MUST be reset when odometry is reset
        m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);
        m_odometry.resetPosition(Rotation2d.fromDegrees(getGyroAngle()), 0.0, 0.0, pose); 
    }
    
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        m_leftMotor.setVoltage(-leftVolts);
        m_rightMotor.setVoltage(rightVolts);// make sure right is negative becuase sides are opposite
        m_robotDrive.feed();
    }
    
    public double getAverageEncoderDistance() {
        return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2.0;
    }
    
    public double getLeftEncoderDistance() {
        return m_leftEncoder.getPosition();
    }
    
    public double getRightEncoderDistance() {
        return m_rightEncoder.getPosition();
    }
    
    public double getHeading() {
        return m_odometry.getPoseMeters().getRotation().getDegrees();
    }
    
    private double getGyroAngle() {
        return Math.IEEEremainder(m_navX.getAngle(), 360) * -1; // -1 wa put here for unknown reason look in documatation
    }
    
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
    }
    
    @Override
    public void periodic() {
        m_odometry.update(Rotation2d.fromDegrees(getGyroAngle()), getLeftEncoderDistance(), getRightEncoderDistance());
        SmartDashboard.putNumber("Heading", getHeading());
        SmartDashboard.putString("Pose", getPose().toString());
    }
    
    
    public void allDrive(double throttle, double rotate, boolean squaredInputs) {
        m_robotDrive.arcadeDrive(MAX_DRIVE_SPEED * throttle, MAX_TURN_SPEED * rotate, squaredInputs);
    }
    
    public double turnSpeedCalc(double angleError) {
        if (Math.abs(angleError) > 60) {
            return 0.8 * Math.signum(angleError);
        } else if (Math.abs(angleError) > 30) {
            return 0.4 * Math.signum(angleError);
        } else if (Math.abs(angleError) > 10) {
            return 0.13 * Math.signum(angleError);
        } else if (Math.abs(angleError) > 5) {
            return 0.07 * Math.signum(angleError);
        } else {
            return 0.065 * Math.signum(angleError);
        }
    }
    
    public double getPitch() {
        return m_navX.getPitch(); 
    }
    
    // public void setIdleMode(IdleMode idleMode) {
    //     if (Robot.isReal()) {
    //         m_leftMotor.
    //         Arrays.asList(m_leftMotor, m_rightMotor)
    //         .forEach((SparkMax spark) -> spark.setIdleMode(idleMode));
    //     }
    // }    
}