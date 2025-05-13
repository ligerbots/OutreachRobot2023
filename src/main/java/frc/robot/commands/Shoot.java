package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;

public class Shoot extends Command {
    public enum State {
        IDLE,
        SPIN_UP, 
        SHOOT
    }
    
    private static final Rotation2d DEFAULT_HOOD_ANGLE = Rotation2d.fromDegrees(45);
    private static final double DEFAULT_SHOOT_SPEED = 1750;
    
    // time (seconds) it takes to shoot the ball after starting the flup
    private static final double SHOOT_TIMER = 1.0;
    private static final double SPIN_UP_TIMER = 3.0;
    
    private final Hood m_hood;
    private Rotation2d m_hoodAngle;
    private final Shooter m_shooter;
    private double m_speed;
    private final Transfer m_transfer;
    
    private State m_state = State.IDLE;
    private final Timer m_timer = new Timer();
    
    public Shoot(Shooter shooter, Hood hood, Transfer transfer, double speed, Rotation2d hoodAngle) {
        m_hood = hood;
        m_hoodAngle = hoodAngle;
        m_shooter = shooter;
        m_speed = speed;
        m_transfer = transfer;
        addRequirements(shooter, hood, transfer);
        SmartDashboard.putNumber("shooter/RPM_TEST", m_speed);
    }
    
    public Shoot(Shooter shooter, Hood hood, Transfer transfer) {
        this(shooter, hood, transfer, DEFAULT_SHOOT_SPEED, DEFAULT_HOOD_ANGLE);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_speed = SmartDashboard.getNumber("shooter/RPM_TEST", DEFAULT_SHOOT_SPEED);
        m_shooter.setShooterRpm(m_speed);
        m_hood.setAngle(m_hoodAngle);
        m_state = State.SPIN_UP;
        SmartDashboard.putString("shooter/state", m_state.toString());
        m_timer.restart();
    }
    
    @Override
    public void execute() {
        if (m_state == State.SPIN_UP) {
            if (m_shooter.speedOnTarget(m_speed, 5) || m_timer.hasElapsed(SPIN_UP_TIMER)) {
                // add start transfer
                m_transfer.intake();
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
        m_transfer.stop();
        m_hood.stow();
        m_state = State.IDLE;
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_state == State.IDLE;
    }
}
