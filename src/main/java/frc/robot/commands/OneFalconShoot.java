package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OneFalconShooter;

public class OneFalconShoot extends Command {
    OneFalconShooter m_shooter;
    DoubleSupplier m_rps;

    // Constants
    final double ACCEPTABLE_RPS_ERROR = 1.0; // TODO: Tune
    
    public OneFalconShoot(OneFalconShooter shooter, DoubleSupplier rps) {
        m_shooter = shooter;
        m_rps = rps;

        addRequirements(m_shooter);
    }

    public void initialize() {
        m_shooter.setShooterMotorRPS(m_rps.getAsDouble());
    }

    public void execute() {
        if (Math.abs(m_rps.getAsDouble() - m_shooter.getShooterMotorRPS()) <= ACCEPTABLE_RPS_ERROR) {
            m_shooter.shoot();
        }
    }

    public void end(boolean interrupted) {
        m_shooter.stopAll();
    }

    public boolean isFinished() {
        return false;
    }
}