package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class OneFalconShooter extends SubsystemBase {
    TalonFX m_shooter;
    CANSparkMax m_flup;

    // PID constants for Shooter
    // TODO: Tune (these are shamelessly scraped from CTRE's website)
    final double SHOOTER_kS = 0.1;
    final double SHOOTER_kV = 0.11;
    final double SHOOTER_kP = 0.12;
    final double SHOOTER_kI = 0.0;
    final double SHOOTER_kD = 0.0;

    // Local Constants
    final double FLYWHEEL_CIRCUMFERENCE = 4 * Math.PI / 12; //feet
    final double SHOOTER_STATOR_CURRENT_LIMIT = 40.0; //Ask Electrical to double-check this value

    public OneFalconShooter() {
        m_shooter = new TalonFX(Constants.SHOOTER_ONE_ID);
        m_flup = new CANSparkMax(Constants.SHOOTER_FLUP_ID, MotorType.kBrushless);

        // PID
        Slot0Configs shooterSlot0 = new Slot0Configs();
        shooterSlot0.kS = SHOOTER_kS;
        shooterSlot0.kV = SHOOTER_kV;
        shooterSlot0.kP = SHOOTER_kP;
        shooterSlot0.kI = SHOOTER_kI;
        shooterSlot0.kD = SHOOTER_kD;

        m_shooter.getConfigurator().apply(shooterSlot0);

        // Supply current limiting
        CurrentLimitsConfigs shooterCurrentLimitConfigs = new CurrentLimitsConfigs();
        shooterCurrentLimitConfigs.StatorCurrentLimit = SHOOTER_STATOR_CURRENT_LIMIT;
        shooterCurrentLimitConfigs.StatorCurrentLimitEnable = true;
        m_shooter.getConfigurator().apply(shooterCurrentLimitConfigs);

        // SmartDashboard logging
        SmartDashboard.putNumber("shooter/shooterStatorCurrentLimit", SHOOTER_STATOR_CURRENT_LIMIT);
        SmartDashboard.putString("shooter/Status", "Idle");
    }

    public void setShooterMotorRPS(double rps) {
        // Gear ratio 1:1 so this is fine for flywheel RPM as well
        final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
        m_shooter.setControl(m_request.withVelocity(rps));
        SmartDashboard.putString("shooter/Status", "Spinning up");
    }

    public double getShooterMotorRPS() {
        return m_shooter.getVelocity().getValueAsDouble();
    }

    public void setFlywheelSurfaceSpeed(double fps) {
        setShooterMotorRPS(fps / FLYWHEEL_CIRCUMFERENCE);
    }

    public double getFlywheelSurfaceSpeed() {
        return getShooterMotorRPS() * FLYWHEEL_CIRCUMFERENCE;
    }

    public void shoot() {
        m_flup.set(0.1); //TODO: Tune
        SmartDashboard.putString("shooter/Status", "Shooting");
    }

    public void stopAll() {
        setShooterMotorRPS(0);
        m_flup.set(0);
    }
}
