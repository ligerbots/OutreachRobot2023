// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.logging.FileBackend;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {    
    private static final double GEAR_RATIO = 24.0 / 42.0; // 1.75:1

    private static final double ABS_ENCODER_OFFSET_HACK = 10;
    // Zero point of the absolute encoder (in rotations)
    private static final double ABS_ENCODER_ZERO_OFFSET = (91.0 + ABS_ENCODER_OFFSET_HACK / GEAR_RATIO) / 360.0; //FIXME: Find proper offset & diagnose scaling error
    
    private static final double MIN_ANGLE_DEG = 0 + ABS_ENCODER_OFFSET_HACK;
    private static final double MAX_ANGLE_DEG = 60 + ABS_ENCODER_OFFSET_HACK;


    private static final double ANGLE_TOLERANCE_DEG = 1.0;

    private static final int CURRENT_LIMIT = 60;
    
    // Constants to limit the hood rotation speed
    // max vel: 1 rotation = 10 seconds and then gear_ratio
    private static final double MAX_VEL_ROT_PER_SEC = 1.5;
    private static final double MAX_ACC_ROT_PER_SEC2 = 3.0;
    private static final double ROBOT_LOOP_PERIOD = 0.02;
    
    // Constants for the hood PID controller
    private static final double K_P = 6.0;
    private static final double K_I = 0.0;
    private static final double K_D = 0.0;
    private static final double K_G = 3.0;
    private static final double FF_OFFSET_DEG = 15.0 + ABS_ENCODER_OFFSET_HACK;

    private final SparkMax m_motor;
    private final SparkAbsoluteEncoder m_absoluteEncoder;
    private final SparkClosedLoopController m_controller;

    // Used for checking if on goal
    private Rotation2d m_goal = Rotation2d.fromDegrees(0 + ABS_ENCODER_OFFSET_HACK);
    private Rotation2d m_goalClipped = Rotation2d.fromDegrees(0 + ABS_ENCODER_OFFSET_HACK);

    // Trapezoid Profile
    private final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(MAX_VEL_ROT_PER_SEC, MAX_ACC_ROT_PER_SEC2));
    private State m_currentState = new State();

    // Construct a new shooterhood subsystem
    public Hood() {
        m_motor = new SparkMax(Constants.SHOOTER_HOOD_CAN_ID, MotorType.kBrushed);

        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(CURRENT_LIMIT);

        AbsoluteEncoderConfig absEncConfig = new AbsoluteEncoderConfig();
        absEncConfig.velocityConversionFactor(GEAR_RATIO / 60.0);   // convert rpm to rps
        absEncConfig.positionConversionFactor(GEAR_RATIO);
        absEncConfig.zeroOffset(ABS_ENCODER_ZERO_OFFSET);
        absEncConfig.inverted(false); 
        // absEncConfig.setSparkMaxDataPortConfig();
        config.apply(absEncConfig);
        
        // set up the PID for MAX Motion
        config.closedLoop.p(K_P).i(K_I).d(K_D);

        config.closedLoop.outputRange(-1, 1);
        config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        config.closedLoop.positionWrappingEnabled(false);  // don't treat it as a circle
        // config.closedLoop.positionWrappingInputRange(0,1.0);
                        
        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_absoluteEncoder = m_motor.getAbsoluteEncoder();

        // controller for PID control
        m_controller = m_motor.getClosedLoopController();

        SmartDashboard.putNumber("hood/testAngle", 0);
    }

    @Override
    public void periodic() {
        m_goal = Rotation2d.fromDegrees(SmartDashboard.getNumber("hood/testAngle", 0));
        m_goalClipped = limitHoodAngle(m_goal);

        State goalState = new State(m_goalClipped.getRotations(), 0);

        // Trapezoid Profile
        m_currentState = m_profile.calculate(ROBOT_LOOP_PERIOD, m_currentState, goalState);

        double feedforward = 0;
        if (m_goalClipped.getDegrees() > 2 + ABS_ENCODER_OFFSET_HACK) {
            // feedforward in Volts
            feedforward = K_G * Math.cos(Math.toRadians(m_goalClipped.getDegrees() - FF_OFFSET_DEG));
        }
        
        m_controller.setReference(m_currentState.position, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward);

        // Display current values on the SmartDashboard
        // This also gets logged to the log file on the Rio and aids in replaying a match
        SmartDashboard.putNumber("hood/goal", m_goal.getDegrees());
        SmartDashboard.putNumber("hood/statePosition", m_currentState.position * 360.0);
        SmartDashboard.putNumber("hood/goalClipped", m_goalClipped.getDegrees());
        SmartDashboard.putNumber("hood/absoluteEncoder", getAngle().getDegrees());
        SmartDashboard.putNumber("hood/outputCurrent", m_motor.getOutputCurrent());
        SmartDashboard.putNumber("hood/busVoltage", m_motor.getBusVoltage());
        SmartDashboard.putBoolean("hood/onGoal", angleWithinTolerance());
        SmartDashboard.putNumber("hood/appliedOutput", m_motor.getAppliedOutput());
        SmartDashboard.putNumber("hood/feedforward", feedforward);
    }

    // get the current hood angle
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(m_absoluteEncoder.getPosition());
    }

    // set shooterhood angle
    public void setAngle(Rotation2d angle) {
        m_goal = angle;
        SmartDashboard.putNumber("hood/goal", m_goal.getDegrees());
    }
    
    // get the angle from the absolute encoder
    // public double getAbsEncoderAngleRadians() {
    //     return TWO_PI * m_absoluteEncoder.getDistance();
    // }

    // needs to be public so that commands can get the restricted angle
    public Rotation2d limitHoodAngle(Rotation2d angle) {
        return Rotation2d.fromDegrees(MathUtil.clamp(angle.getDegrees(), MIN_ANGLE_DEG, MAX_ANGLE_DEG));
    }

    public boolean angleWithinTolerance() {
        return Math.abs(m_goalClipped.minus(getAngle()).getDegrees()) < ANGLE_TOLERANCE_DEG;
    }
}