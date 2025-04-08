// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

import com.revrobotics.CANSparkBase.IdleMode;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.kauailabs.navx.frc.AHRS; 
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.math.util.Units;

public class DriveTrain extends SubsystemBase {
    
    private CANSparkMax m_leftMotor = new CANSparkMax(Constants.LEFT_MOTOR_CAN_ID, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    private CANSparkMax m_rightMotor = new CANSparkMax(Constants.RIGHT_MOTOR_CAN_ID, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    
    private DifferentialDrive m_robotDrive;
    private DifferentialDriveOdometry m_odometry;
    
    private RelativeEncoder m_leftEncoder; 
    private RelativeEncoder m_rightEncoder; 
    
    private final AHRS m_navX = new AHRS(Port.kMXP, (byte) 200);
    
    private static final int CURRENT_LIMIT = 35;
    
    private static final double DRIVE_GEAR_REDUCTION = 1/19.527; //TODO: Double check number
    private static final double METER_PER_REVOLUTION = (Units.inchesToMeters(8)*Math.PI) * DRIVE_GEAR_REDUCTION;
    
    public DriveTrain() {
        m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
        
        // Set current limiting on drve train to prevent brown outs
        m_leftMotor.setSmartCurrentLimit(CURRENT_LIMIT);
        m_rightMotor.setSmartCurrentLimit(CURRENT_LIMIT);
        
        // Set motors to brake when idle. We don't want the drive train to coast.
        m_leftMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);
        m_rightMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);
        
        m_leftEncoder.setPositionConversionFactor(METER_PER_REVOLUTION);
        m_rightEncoder.setPositionConversionFactor(METER_PER_REVOLUTION);

        m_leftEncoder = m_leftMotor.getEncoder();
        m_rightEncoder = m_rightMotor.getEncoder();
        
        //FIXME: Not sure what the second two arguments do, from looking at the docs they seem unnessary
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
        // TODO: We should look into using the deadband settings in DifferentialDrive
        if (squaredInputs) {
            if (Math.abs(throttle) < 0.1)
            throttle = 0;
            if (Math.abs(rotate) < 0.1)
            rotate = 0;
        }
        m_robotDrive.arcadeDrive(throttle, -rotate, squaredInputs);
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
    
    public void setIdleMode(IdleMode idleMode) {
        if (Robot.isReal()) {
            Arrays.asList(m_leftMotor, m_rightMotor)
            .forEach((CANSparkMax spark) -> spark.setIdleMode(idleMode));
        }
    }
    
    
}