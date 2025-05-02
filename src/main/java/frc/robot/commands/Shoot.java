package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
    public enum State {
        IDLE,
        SPIN_UP, 
        SHOOT
    }
    
    private static final double DEFAULT_SHOOT_SPEED = 1000;
    
    // time (seconds) it takes to shoot the ball after starting the flup
    private static final double SHOOT_TIMER = 0.25;
    private static final double SPIN_UP_TIMER = 2;
    
    private final Shooter m_shooter;
    private final double m_speed;
    
    private State m_state = State.IDLE;
    private final Timer m_timer = new Timer();
    
    public Shoot(Shooter shooter, double speed) {
        m_shooter = shooter;
        m_speed = speed;
        addRequirements(shooter);
    }
    
    public Shoot(Shooter shooter) {
        this(shooter, DEFAULT_SHOOT_SPEED);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_shooter.setShooterRpm(m_speed);
        m_state = State.SPIN_UP;
        SmartDashboard.putString("shooter/state", m_state.toString());
        m_timer.restart();
    }
    
    @Override
    public void execute() {
        if (m_state == State.SPIN_UP) {
            if (m_shooter.speedOnTarget(m_speed, 5) || m_timer.hasElapsed(SPIN_UP_TIMER)) {
                // start flup to shoot
                m_shooter.shoot();
                m_timer.restart();
                m_state = State.SHOOT;
            }
        } else if (m_state == State.SHOOT) {
            if (m_timer.hasElapsed(SHOOT_TIMER)) {
                m_shooter.stopAll();
                m_state = State.IDLE;
            }
        }
        
        SmartDashboard.putString("shooter/state", m_state.toString());
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shooter.stopAll();
        m_state = State.IDLE;
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_state == State.IDLE;
    }
}
