package frc.robot.commands;

import java.util.Map.Entry;
import java.util.TreeMap;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
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
    
    // Lookup tables for distance-based shooter control
    private final TreeMap<Double, Double[]> m_distanceLookUp = new TreeMap<Double, Double[]>();
    private final TreeMap<Double, Double> m_hoodAngleLookup = new TreeMap<Double, Double>();
    
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
        
        // Initialize shooter lookup table with example values
        // Distance (feet) -> [RPM, ...]
        m_distanceLookUp.put(5.0, new Double[]{2000.0});
        m_distanceLookUp.put(10.0, new Double[]{2500.0});
        m_distanceLookUp.put(15.0, new Double[]{3000.0});
        m_distanceLookUp.put(20.0, new Double[]{3500.0});
        m_distanceLookUp.put(25.0, new Double[]{4000.0});
        
        // Initialize hood angle lookup table
        // Distance (feet) -> Hood Angle (degrees)
        m_hoodAngleLookup.put(5.0, 15.0);
        m_hoodAngleLookup.put(10.0, 20.0);
        m_hoodAngleLookup.put(15.0, 25.0);
        m_hoodAngleLookup.put(20.0, 28.0);
        m_hoodAngleLookup.put(25.0, 30.0);
        
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

    public double calculateShooterSpeed(double distance) {
        Entry<Double, Double[]> floorEntry = m_distanceLookUp.floorEntry(distance);
        Entry<Double, Double[]> ceilingEntry = m_distanceLookUp.higherEntry(distance);

        if (floorEntry == null || ceilingEntry == null) {
            DriverStation.reportError("Shoot: floorEntry or ceilingEntry was null", false);
            // Typical speed. Not sure this will work for much, but it won't break anything.
            return 3000.0;
        }

        double ratio = MathUtil.inverseInterpolate(floorEntry.getKey(), ceilingEntry.getKey(), distance);
        double rpmTarget = MathUtil.interpolate(floorEntry.getValue()[0], ceilingEntry.getValue()[0], ratio);

        System.out.format("Shoot: ratio %3.2f, floor %4.1f, dist %4.1f, ceiling %4.1f, RPM %4.1f",
                ratio, floorEntry.getKey(), distance, ceilingEntry.getKey(), rpmTarget);
        return rpmTarget;
    }

    public double calculateHoodAngle(double distance) {
        Entry<Double, Double> floorEntry = m_hoodAngleLookup.floorEntry(distance);
        Entry<Double, Double> ceilingEntry = m_hoodAngleLookup.higherEntry(distance);

        if (floorEntry == null || ceilingEntry == null) {
            DriverStation.reportError("Shoot: Hood floorEntry or ceilingEntry was null", false);
            // Default hood angle
            return 20.0;
        }

        double ratio = MathUtil.inverseInterpolate(floorEntry.getKey(), ceilingEntry.getKey(), distance);
        double hoodAngle = MathUtil.interpolate(floorEntry.getValue(), ceilingEntry.getValue(), ratio);

        System.out.format("Shoot: Hood ratio %3.2f, floor %4.1f, dist %4.1f, ceiling %4.1f, angle %4.1f",
                ratio, floorEntry.getKey(), distance, ceilingEntry.getKey(), hoodAngle);
        return hoodAngle;
    }
}
