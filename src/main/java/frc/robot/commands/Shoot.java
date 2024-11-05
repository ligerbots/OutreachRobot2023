package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
//import frc.robot.subsystems.Vision.VisionMode;

public class Shoot extends Command {

  /**
   * Creates a new ShooterCommand.
   */

  double waitTime;
  double startTime;

  Shooter m_shooter;
  Turret m_turret;
  DriveTrain m_robotDrive;
  ShooterPIDTuner pidTuner;
  double shooterTargetSpeed;

  boolean startShooting;

  TurretCommand m_turretCommand;
  //DriveCommand driveCommand;

  int initialTurretTicks;

  double stableRPMTime;
  boolean startedTimerFlag;
  boolean foundTarget;
  boolean setPid;
 
  public enum ControlMethod {
    ACQUIRING, // Acquiring vision target
    SPIN_UP, // PIDF to desired RPM
    HOLD_WHEN_READY, // calculate average kF
    HOLD, // switch to pure kF control
  }

  ControlMethod currentControlMode;
  boolean rescheduleDriveCommand;

  // I don't know what these are doing here, but I don't think they're relevant. I may delete it if I confirm that this is unusable.
  // private TalonFX m_leftLeader = new TalonFX(Constants.LEADER_LEFT_CAN_ID);
  // private TalonFX m_leftFollower = new TalonFX(Constants.FOLLOWER_LEFT_CAN_ID);
  // private TalonFX m_rightLeader = new TalonFX(Constants.LEADER_RIGHT_CAN_ID);
  // private TalonFX m_rightFollower = new TalonFX(Constants.FOLLOWER_RIGHT_CAN_ID);

  // private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftLeader, m_leftFollower);
  // private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightLeader, m_rightFollower);

  public Shoot(Shooter shooter, Turret turret, DriveTrain robotDrive, TurretCommand turretCommand, /*DriveCommand driveCommand,*/ boolean rescheduleDriveCommand) {
    m_shooter = shooter;
    addRequirements(shooter);
    m_turret = turret;
    // The following statement will cause the CarouselCommand to be interrupted. This is good.
    // addRequirements(carousel);
    m_robotDrive = robotDrive;
    m_turretCommand = turretCommand;
    // System.out.println("Shooter.carouselCommand = " + this.carouselCommand);
    // this.driveCommand = driveCommand;
    this.rescheduleDriveCommand = rescheduleDriveCommand;
    startShooting = false;
    pidTuner = new ShooterPIDTuner(shooter);

    // setup PID control for TalonFX
      
    /*m_leftLeader.configFactoryDefault();
    m_leftLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    m_leftLeader.set(ControlMode.Position,0);
    m_leftLeader.config_kP(0, 50);
    m_leftLeader.config_kI(0, 0);
    m_leftLeader.config_kD(0, 0);
    m_leftLeader.config_kF(0, 0);
    m_leftLeader.setSensorPhase(false);

    m_rightLeader.configFactoryDefault();
    m_rightLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    m_rightLeader.set(ControlMode.Position,0);
    m_rightLeader.config_kP(0, 50);
    m_rightLeader.config_kI(0, 0);
    m_rightLeader.config_kD(0, 0);
    m_rightLeader.config_kF(0, 0);
    m_rightLeader.setSensorPhase(true);
        
    m_rightMotors.setInverted(true);

    setMotorMode(NeutralModeValue.Coast);*/
  }

  /*public void setMotorMode(NeutralModeValue m) {
    m_leftLeader.setNeutralMode(m);
    m_leftFollower.setNeutralMode(m);
    m_rightLeader.setNeutralMode(m);
    m_rightFollower.setNeutralMode(m); 
  }*/

  public void rapidFire() {
    m_shooter.shoot();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    SmartDashboard.putString("shooter/Shooting", "Shoot");

    foundTarget = false;
    shooterTargetSpeed = 0.0;
    // This flag is used so we only set the PID values once per command. We don't want to constantly reset the PID
    // values  in the execute() method.
    setPid = true;

    // driveCommand.cancel();
    startTime = Robot.time();
    // m_shooter.m_vision.setMode(VisionMode.GOALFINDER);
    m_turretCommand.cancel();
    currentControlMode = ControlMethod.ACQUIRING;
    //starts spinning up the shooter to hard-coded PID values
    pidTuner.spinUpTune();
    System.out.println("Initial NavX Heading: " + m_robotDrive.getHeading());
    // store current turretTick value
    initialTurretTicks = (int) m_turret.getPosition();

    // angleError = m_shooter.m_vision.getRobotAngle();
    // distance = m_shooter.m_vision.getDistance();

    currentControlMode = ControlMethod.SPIN_UP;
    startedTimerFlag = false;
    System.out.println("Initial Angle Offset: " + angleError);
    // shooter.setTurretAdjusted(0.0/*-Robot.angleErrorAfterTurn*/);
  }

  // Called every time the scheduler runs while the command is scheduled.
  double angleError;
  double distance;

  boolean speedOnTarget = false;
  boolean hoodOnTarget = false;
  boolean angleOnTarget = false;

  @Override
  public void execute() {
    if (!foundTarget) {
      // distance = m_shooter.m_vision.getDistance();
      if (distance != 0.0) {
        foundTarget = true;
        currentControlMode = ControlMethod.SPIN_UP;
        // We found the target. Set the turret angle based on the vision system before
        // we spin up the shooter
        // angleError = m_shooter.m_vision.getRobotAngle();
        // angleError = 0.0;
        m_shooter.setTurretAdjusted(angleError);
        shooterTargetSpeed = -m_shooter.calculateShooterSpeed(distance);  
        m_shooter.prepareShooter(distance);   
      }   
    }

    //System.out.println("Target Speed: " + shooter.calculateShooterSpeed(distance) + "   Current Speed: " + shooter.getSpeed() + " ");

    if (currentControlMode == ControlMethod.SPIN_UP){ 

      if (m_shooter.speedOnTarget(shooterTargetSpeed, 15)) {
        if (startedTimerFlag) {
          if (Robot.time() - stableRPMTime > 0.2) {
            currentControlMode = ControlMethod.HOLD;
          }
        } else {
          stableRPMTime = Robot.time();
          startedTimerFlag = true;
        }
      }
      else {
        startedTimerFlag = false;
      }
    }
    else if (currentControlMode == ControlMethod.HOLD) {
      if(setPid){
        pidTuner.HoldTune();
      }
      setPid = false;
    }

  
    speedOnTarget = (m_shooter.speedOnTarget(shooterTargetSpeed, 8) && currentControlMode == ControlMethod.HOLD) || Robot.time() - startTime > 3.5; //TODO: May need to adjust acceptable error
    hoodOnTarget = Robot.time() - startTime > 0.75;//shooter.hoodOnTarget(shooter.calculateShooterHood(distance));

    // !carousel.backwards will need to be removed when the shooter is re-written
    if (speedOnTarget && hoodOnTarget && !m_turret.backwards)
        rapidFire();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopAll();
    // m_shooter.m_vision.setMode(VisionMode.INTAKE);
    m_turretCommand.schedule();
    System.out.println("Shooter: turretCommand scheduled" + m_turretCommand);
    //if (rescheduleDriveCommand) {
     // driveCommand.schedule();
    //}
    SmartDashboard.putString("shooter/Shooting", "Idle");
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Robot.isSimulation()) return (Robot.time() - startTime) > 2.0;

    // TODO: this should just check to see if the turret has rotated 5 TURRET_FIFTH_ROTATION_TICKS intervals
    return ((int) m_turret.getPosition() - initialTurretTicks) < -5 * Constants.TURRET_FIFTH_ROTATION_TICKS || (distance == 0.0 && Robot.time() - startTime > 2.0);
  }

  public class ShooterPIDTuner {
    private double p,i,d;
    private Shooter shooter;
    public ShooterPIDTuner(Shooter shooter){
      this.shooter = shooter;
    }
    
    public void spinUpTune(){
      shooter.calibratePID(0.000145, 1e-8, 0.0);
    }
    
    public void HoldTune(){
      getPIDFromDashBoard();
      setPID();
    }
    
    public void getPIDFromDashBoard(){
      p = SmartDashboard.getNumber("shooter/P", 0.000145);
      i = SmartDashboard.getNumber("shooter/I",1e-8);
      d = SmartDashboard.getNumber("shooter/D", 0);
    }
    
    public void setPID(){
      shooter.calibratePID(p, i, d);
    }
  }
}
