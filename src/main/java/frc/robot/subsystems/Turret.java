// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
    
    private static final double MIN_ANGLE_DEG = 100.0;
    private static final double MAX_ANGLE_DEG = 200.0;

    // NOTE: All constants were taken from the 2023 arm 
    // Note: Current values for limits are refrenced with the shooter being flat
    // facing fowards as zero.
    // As of writing the above note we still may want to change the limits
    public static final double ANGLE_TOLERANCE_DEG = 1.0;

    private static final int CURRENT_LIMIT = 60;
    
    // 25:1 planetary plus 42:18 sprockets
    // private static final double GEAR_RATIO = 25.0 * (42.0 / 18.0);
      
    // Constants to limit the shooterPivot rotation speed
    // max vel: 1 rotation = 10 seconds  and then gear_ratio
    private static final double MAX_VEL_ROT_PER_SEC = 1.5;
    private static final double MAX_ACC_ROT_PER_SEC2 = 3.0;
    private static final double ROBOT_LOOP_PERIOD = 0.02;

    // Zero point of the absolute encoder
    private static final double ABS_ENCODER_ZERO_OFFSET = (225.1-180.0)/360.0; // FIXME: CHANGE 'ABS_ENCODER_ZERO_OFFSET' to a real value (this is from 2025)
    //0.5/360.0; //(135.2+180)/360.0; 
    
    // Constants for the pivot PID controller
    private static final double K_P = 5.0;
    private static final double K_I = 0.0;
    private static final double K_D = 0.0;

    private final SparkMax m_motor;
    private final SparkAbsoluteEncoder m_absoluteEncoder;
    private final SparkClosedLoopController m_controller;

    // Used for checking if on goal
    private Rotation2d m_goal = Rotation2d.fromDegrees(0);
    private Rotation2d m_goalClipped = Rotation2d.fromDegrees(0);

    // adjustment offset. Starts at 0, but retained throughout a match
    // private double m_angleAdjustment = Math.toRadians(0.0);

    // Trapezoid Profile
    private final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(MAX_VEL_ROT_PER_SEC, MAX_ACC_ROT_PER_SEC2));
    private State m_currentState = new State();

    // Construct a new shooterPivot subsystem
    public Turret() {

        m_motor = new SparkMax(Constants.SHOOTER_HOOD_CAN_ID, MotorType.kBrushed);

        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(CURRENT_LIMIT);

        AbsoluteEncoderConfig absEncConfig = new AbsoluteEncoderConfig();
        absEncConfig.velocityConversionFactor(1/60.0);   // convert rpm to rps
        absEncConfig.zeroOffset(ABS_ENCODER_ZERO_OFFSET);
        absEncConfig.inverted(false); //TODO: Verif
        // absEncConfig.setSparkMaxDataPortConfig();
        config.apply(absEncConfig);
        
        // set up the PID for MAX Motion
        // config.closedLoop.pidf(K_P, K_I, K_D, K_FF);
        config.closedLoop.p(K_P).i(K_I).d(K_D);
        //  K_FF);

        config.closedLoop.outputRange(-1, 1);
        config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        config.closedLoop.positionWrappingEnabled(false);  // don't treat it as a circle
        // config.closedLoop.positionWrappingInputRange(0,1.0);
                        
        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // motor encoder - set calibration and offset to match absolute encoder
        // m_encoder = m_motor.getEncoder();

        m_absoluteEncoder = m_motor.getAbsoluteEncoder();

        // controller for PID control
        m_controller = m_motor.getClosedLoopController();

        // updateMotorEncoderOffset();
        resetGoal();

        SmartDashboard.putBoolean("pivot/coastMode", false);
        setCoastMode();

        SmartDashboard.putNumber("pivot/testAngle", 0);
    }

    @Override
    public void periodic() {

        // double oldGoalClipped = m_goalClipped.getDegrees();
        m_goalClipped = limitPivotAngle(m_goal);

        // boolean goalChanged = Math.abs(m_goalClipped.getDegrees() - oldGoalClipped) > 5.0;

        // double feedforward = -K_GRAVITY * Math.sin(angle.getRadians() - GRAVITY_ZERO);
        State goalState = new State(m_goalClipped.getRotations(), 0);

        // Trapezoid Profile
        m_currentState = m_profile.calculate(ROBOT_LOOP_PERIOD, m_currentState, goalState);

        double angleDeg = getAngle().getDegrees();

        // feedforward in Volts
        double feedforward = 0;

        m_controller.setReference(m_currentState.position, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward);

        // Display current values on the SmartDashboard
        // This also gets logged to the log file on the Rio and aids in replaying a match
        SmartDashboard.putNumber("pivot/statePosition", m_currentState.position * 360.0);
        SmartDashboard.putNumber("pivot/goalClipped", m_goalClipped.getDegrees());
        SmartDashboard.putNumber("pivot/absoluteEncoder", angleDeg);
        SmartDashboard.putNumber("pivot/outputCurrent", m_motor.getOutputCurrent());
        SmartDashboard.putNumber("pivot/busVoltage", m_motor.getBusVoltage());
        SmartDashboard.putBoolean("pivot/onGoal", angleWithinTolerance());
        SmartDashboard.putNumber("pivot/appliedOutput", m_motor.getAppliedOutput());
        SmartDashboard.putNumber("pivot/velocity", getVelocity().getDegrees());
        // SmartDashboard.putNumber("pivot/feedforward", feedforward);
        // SmartDashboard.putNumber("pivot/accel", accel);
    }

    // get the current pivot angle
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(m_absoluteEncoder.getPosition());
    }

    public Rotation2d getVelocity() {
        // Encoder returns RPM
        return Rotation2d.fromRotations(m_absoluteEncoder.getVelocity() * 60);
    }

    public void run(double speed) {
        m_motor.set(speed);
    }

    // set shooterPivot angle
    public void setAngle(Rotation2d angle) {
        m_goal = angle;
        SmartDashboard.putNumber("pivot/goal", m_goal.getDegrees());
    }
    
    // get the angle from the absolute encoder
    // public double getAbsEncoderAngleRadians() {
    //     return TWO_PI * m_absoluteEncoder.getDistance();
    // }

    // needs to be public so that commands can get the restricted angle
    public Rotation2d limitPivotAngle(Rotation2d angle) {
        return Rotation2d.fromDegrees(MathUtil.clamp(angle.getDegrees(), MIN_ANGLE_DEG, MAX_ANGLE_DEG));
    }

    public boolean angleWithinTolerance() {
        //TODO does MAXMotion provide this?
        return Math.abs(m_goalClipped.minus(getAngle()).getDegrees()) < ANGLE_TOLERANCE_DEG;
    }

    // public void adjustAngle(boolean goUp) {
    //     double adjust = (goUp ? 1 : -1) * ADJUSTMENT_STEP;
    //     m_angleAdjustment += adjust;
    //     setAngle(m_goalRadians + adjust, false);
    // }

    public void resetGoal() {
        Rotation2d angle = getAngle();
        setAngle(angle);
        m_currentState.position = angle.getRotations();
        m_currentState.velocity = 0;
    }

    public void setCoastMode() {
        boolean coastMode = SmartDashboard.getBoolean("shooterPivot/coastMode", false);
        if (coastMode) {
            m_motor.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            m_motor.stopMotor();
        } else
        m_motor.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}