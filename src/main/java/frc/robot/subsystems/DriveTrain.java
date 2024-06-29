// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;
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
    private static final double JOYSTICK_DEADBAND = 0.05;

    // check what we should add in constants and what in subsystems
    // CANSparkMax(int, CANSparkLowLevel.MotorType)
    private CANSparkMax m_leftMotor = new CANSparkMax(Constants.LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
    private CANSparkMax m_rightMotor = new CANSparkMax(Constants.RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);

    private DifferentialDrive m_robotDrive;
    private DifferentialDriveOdometry m_odometry;

    private RelativeEncoder m_leftEncoder; // = new Encoder(LEFT_ENCODER_PORTS);//(LOOK AT THIS!!)
    private RelativeEncoder m_rightEncoder; // = new Encoder(RIGHT_ENCODER_PORTS);
    // private final static int[] LEFT_ENCODER_PORTS = new int[]{0, 1};
    // private final static int[] RIGHT_ENCODER_PORTS = new int[]{2,3};

    AHRS m_navX;

    double m_limitedThrottle;

    /*
     * Simulation
     * public DifferentialDrivetrainSim drivetrainSimulator;
     * private EncoderSim leftEncoderSim;
     * private EncoderSim rightEncoderSim;
     * // The Field2d class simulates the field in the sim GUI. Note that we can
     * have only one
     * // instance!
     * private Field2d fieldSim;
     * private SimDouble gyroAngleSim;
     */

    // PID
    private SparkPIDController m_leftController;
    private SparkPIDController m_rightController;

    private static double K_P = 1.0;
    private static double K_I = 0.0;
    private static double K_D = 0.0;
    private static double K_FF = 1.0;

    private static final double DRIVE_GEAR_REDUCTION = 1.0 / 19.527; // TODO: check; should be written as gear tooth ratios, not the final number
    private static final double METER_PER_REVOLUTION = (Units.inchesToMeters(8) * Math.PI) * DRIVE_GEAR_REDUCTION;

    public DriveTrain() {
        m_leftController = m_leftMotor.getPIDController();
        m_leftController.setP(K_P);
        m_leftController.setI(K_I);
        m_leftController.setD(K_D);
        m_leftController.setFF(K_FF);

        m_rightController = m_rightMotor.getPIDController();
        m_rightController.setP(K_P);
        m_rightController.setI(K_I);
        m_rightController.setD(K_D);
        m_rightController.setFF(K_FF);

        // run left motor as inverted, so that positive command goes forward
        m_leftMotor.setInverted(true);

        // Set current limiting on drve train to prevent brown outs
        m_leftMotor.setSmartCurrentLimit(35);
        m_rightMotor.setSmartCurrentLimit(35);

        // Set motors to brake when idle. We don't want the drive train to coast.
        m_leftMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);
        m_rightMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);

        m_leftEncoder.setPositionConversionFactor(METER_PER_REVOLUTION);
        m_rightEncoder.setPositionConversionFactor(METER_PER_REVOLUTION);

        m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
        m_robotDrive.setDeadband(JOYSTICK_DEADBAND);

        // initialize the odometry, giving it the current actual readings
        m_odometry = new DifferentialDriveOdometry(getGyroAngle(), getLeftEncoderDistance(), getRightEncoderDistance());

        /*
         * if (RobotBase.isSimulation()) {
         * // If our robot is simulated
         * // This class simulates our drivetrain's motion around the field.
         * drivetrainSimulator = new DifferentialDrivetrainSim(
         * Constants.kDrivetrainPlant,
         * Constants.kDriveGearbox,
         * Constants.kDriveGearing,
         * Constants.kTrackwidth,
         * Constants.kWheelDiameterMeters / 2.0);
         * 
         * // The encoder and gyro angle sims let us set simulated sensor readings
         * leftEncoderSim = new EncoderSim(m_leftEncoder);
         * rightEncoderSim = new EncoderSim(m_rightEncoder);
         * 
         * // get the angle simulation variable
         * // SimDevice is found by name and index, like "name[index]"
         * gyroAngleSim = new SimDeviceSim("AHRS[" + SPI.Port.kMXP.value +
         * "]").getDouble("Angle");
         * 
         * // the Field2d class lets us visualize our robot in the simulation GUI.
         * fieldSim = new Field2d();
         * 
         * SmartDashboard.putNumber("moveAroundField/startPos", prevStartLocation);
         * SmartDashboard.putNumber("moveAroundField/ballPos", prevBallLocation);
         * }
         */
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        /*
         * if (RobotBase.isSimulation()) {
         * // This is a bit hokey, but if the Robot jumps on the field, we need
         * // to reset the internal state of the DriveTrainSimulator.
         * // No method to do it, but we can reset the state variables.
         * // NOTE: this assumes the robot is not moving, since we are not resetting
         * // the rate variables.
         * drivetrainSimulator.setState(new Matrix<>(Nat.N7(), Nat.N1()));
         * 
         * // reset the GyroSim to match the driveTrainSim
         * // do it early so that "real" odometry matches this value
         * gyroAngleSim.set(-drivetrainSimulator.getHeading().getDegrees());
         * fieldSim.setRobotPose(pose);
         * }
         */

        m_odometry.resetPosition(getGyroAngle(), getLeftEncoderDistance(), getRightEncoderDistance(), pose);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        // we inverted the left motor, so positive volts should be forward for both sides.
        m_leftMotor.setVoltage(leftVolts);
        m_rightMotor.setVoltage(rightVolts);
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

    private Rotation2d getGyroAngle() {
        // navX.getRotation2d() should return the angle going in the correct direction
        return m_navX.getRotation2d();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
    }

    @Override
    public void periodic() {
        m_odometry.update(getGyroAngle(), getLeftEncoderDistance(), getRightEncoderDistance());
        SmartDashboard.putNumber("Heading", getHeading());
    }

    /*
     * @Override
     * public void simulationPeriodic() {
     * // To update our simulation, we set motor voltage inputs, update the
     * simulation,
     * // and write the simulated positions and velocities to our simulated encoder
     * and gyro.
     * // We negate the right side so that positive voltages make the right side
     * // move forward.
     * drivetrainSimulator.setInputs(-m_leftMotor.get() *
     * RobotController.getBatteryVoltage(),
     * m_rightMotors.get() * RobotController.getBatteryVoltage());
     * drivetrainSimulator.update(0.020);
     * 
     * leftEncoderSim.setDistance(drivetrainSimulator.getState(
     * DifferentialDrivetrainSim.State.kLeftPosition));
     * leftEncoderSim.setRate(drivetrainSimulator.getState(DifferentialDrivetrainSim
     * .State.kLeftVelocity));
     * 
     * rightEncoderSim.setDistance(drivetrainSimulator.getState(
     * DifferentialDrivetrainSim.State.kRightPosition));
     * rightEncoderSim.setRate(drivetrainSimulator.getState(
     * DifferentialDrivetrainSim.State.kRightVelocity));
     * 
     * gyroAngleSim.set(-drivetrainSimulator.getHeading().getDegrees());
     * 
     * fieldSim.setRobotPose(getPose());
     * }
     */

    public void drive(double throttle, double rotate, boolean squaredInputs) {
        // deadband of values is handled by the robotDrive class
        m_robotDrive.arcadeDrive(throttle, rotate, squaredInputs);
    }

    public void setIdleMode(IdleMode idleMode) {
        if (Robot.isReal()) {
            Arrays.asList(m_leftMotor, m_rightMotor)
                    .forEach((CANSparkMax spark) -> spark.setIdleMode(idleMode));
        }
    }
}