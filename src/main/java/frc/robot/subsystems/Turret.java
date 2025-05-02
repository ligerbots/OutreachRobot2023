// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
    public boolean backwards;
    public SparkMax m_turretMotor;
    private final RelativeEncoder m_turretMotorEncoder;
    private final double TURRET_GEAR_REDUCTION = 1.0 / 162.5;
    private final double DEG_PER_REVOLUTION = 360 * TURRET_GEAR_REDUCTION;

    // PID Stuff
    private SparkPIDController m_pidController;
    private static double K_P = 1.0; // TODO: Tune
    private static double K_I = 0.0;
    private static double K_D = 0.0;
    private static double K_FF = 1.0;

    //Turret Rotation Limits
    private final int TURRET_LOWER_BOUND_DEGREES = -200;
    private final int TURRET_UPPER_BOUND_DEGREES = 200;
    private final double TURRET_ROTATION_VELOCITY = 1; //TODO: Tune

    //Zeroing Constants
    final int STRING_POT_MAX_LENGTH = 10; //(inches) TODO: Update with the maximum length that the string pot could be before the wires go taut
    final int MOVE_SIZE = 5; //(degrees)

    /** Creates a new Turret. */
    public Turret() {
        // super(null); //TODO: put constraints in here if we're changing this to trapezoidal
        m_turretMotor = new SparkMax(Constants.TURRET_MOTOR_CAN_ID, MotorType.kBrushless);
        
        SparkMaxConfig config = new SparkMaxConfig();
        // Configure the motor
        config.idleMode(IdleMode.kBrake);
        config.encoder.positionConversionFactor(DEG_PER_REVOLUTION); // All future angle references will be based off this
        
        // Set soft limits
        config.enableSoftLimit(SparkMax.SoftLimitDirection.kForward, true);
        config.enableSoftLimit(SparkMax.SoftLimitDirection.kReverse, true);
        config.softLimit(SparkMax.SoftLimitDirection.kForward, TURRET_UPPER_BOUND_DEGREES);
        config.softLimit(SparkMax.SoftLimitDirection.kReverse, TURRET_LOWER_BOUND_DEGREES);

        // Configure PID
        config.pid.p(K_P);
        config.pid.i(K_I);
        config.pid.d(K_D);
        config.pid.ff(K_FF);
        config.pid.positionPIDWrappingEnabled(false);
        
        m_turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        m_turretMotorEncoder = m_turretMotor.getEncoder();
        m_pidController = m_turretMotor.getPIDController();
        
        m_pidController.setReference(0, SparkPIDController.ControlType.kPosition);
        //doZeroRoutine(); TODO: Find out where to put this. Maybe command?
    }

    // protected void useState(TrapezoidProfile.State setPoint) {}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Turret/Angle", m_turretMotorEncoder.getPosition());
    }

    public void doZeroRoutine() {
        //TODO: Put in proper encoders and zero sensor
        /*This is all psuedocode
        String Pot Method:
        initialStringPotLen = m_stringPot.getPosition()
        m_pidController.setReference(MOVE_SIZE)
        finalStringPotLen = m_stringPot.getPosition()
        if initialStringPotLen > finalStringPotLen {
            m_turretMotor.setVelocity(0.1) //very slow
            while m_zeroSensor == False && {
                //Wait for it to continue rotating
            }
            //Once it hits the magnet sensor, set to zero
            m_turretMotorEncoder.setPosition(0);
        } else {
            m_turretMotor.setVelocity(-0.1)
            while m_zeroSensor == False {
                
            }
            m_turretMotorEncoder.setPosition(0);
        }

        String Hard Stop Method: Attach string analog for wires, a bit shorter so it goes tight before the wires.
        The fixed point is set when the string gets pulled taut and the amperage spikes. Con: The string may stretch over time, so maybe use a Hall Effect sensor if it becomes a problem.
        getMotorAmperage(): Get the amperage that the Turret motor is using
        final double TURRET_MOTOR_HARD_STOP_AMPERAGE = 20 Amps (just a guess)
        final double TURRET_ROTATION_VELOCITY = 1 (degrees/second???)
        while (getMotorAmperage() < TURRET_MOTOR_HARD_STOP_AMPERAGE) {
            Do nothing
        }
        Set to positive bound
        m_turretMotorEncoder.setPosition(200);
        */
    }

    public void setZero() {
        m_turretMotorEncoder.setPosition(0);
    }

    public void setTurretAngle(double angle) {
        while (angle <= TURRET_LOWER_BOUND_DEGREES + 10) { //10 degrees shy of bounds to avoid damage from PID overshoot
            angle += 360.0;
        }
        while (angle >= TURRET_UPPER_BOUND_DEGREES - 10) {
            angle -= 360.0;
        }
        m_pidController.setReference(angle, SparkPIDController.ControlType.kPosition);
    }

    public void setTurretVelocity(int direction) {
        m_turretMotor.set(TURRET_ROTATION_VELOCITY * direction);
    }

    public double getPosition() {
        return m_turretMotorEncoder.getPosition();
    }
}
