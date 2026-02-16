  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  private final double MAX_VELOCITY = 3;
  private final double DISTANCE_WHEELS = 0.75;

  private SparkMax leftMaster = new SparkMax(1, MotorType.kBrushless);
  private SparkMax rightMaster = new SparkMax(2, MotorType.kBrushless);
  private SparkMax leftSlave = new SparkMax(3, MotorType.kBrushless);
  private SparkMax rightSlave = new SparkMax(4, MotorType.kBrushless);

  private WPI_PigeonIMU gyro = new WPI_PigeonIMU(5);

  private LTVUnicycleController controller = new LTVUnicycleController(
    VecBuilder.fill(0.0625, 0.125, 2),
    VecBuilder.fill(1, 2),
    0.02,
    MAX_VELOCITY
  );

  private DifferentialDrive diffDrive = new DifferentialDrive(leftMaster, rightMaster);
  private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(
    gyro.getRotation2d(),
    0,
    0
  );
  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DISTANCE_WHEELS);

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    // Spark's configuration
    var leftMasterConfig = new SparkMaxConfig();
    leftMasterConfig.encoder.positionConversionFactor(Math.PI * Units.inchesToMeters(6) / 10.71);
    leftMasterConfig.idleMode(IdleMode.kBrake);
    leftMasterConfig.inverted(false);
    leftMaster.configure(leftMasterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    var rightMasterConfig = new SparkMaxConfig();
    rightMasterConfig.encoder.positionConversionFactor(Math.PI * Units.inchesToMeters(6) / 10.71);
    rightMasterConfig.idleMode(IdleMode.kBrake);
    rightMasterConfig.inverted(true);
    rightMaster.configure(rightMasterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    var leftSlaveConfig = new SparkMaxConfig();
    leftSlaveConfig.follow(leftMaster.getDeviceId());
    leftSlaveConfig.idleMode(IdleMode.kBrake);
    leftSlaveConfig.inverted(false);
    leftSlave.configure(leftSlaveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    var rightSlaveConfig = new SparkMaxConfig();
    rightSlaveConfig.follow(rightMaster.getDeviceId());
    rightSlaveConfig.idleMode(IdleMode.kBrake);
    rightSlaveConfig.inverted(true);
    rightSlave.configure(rightSlaveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  // drives the robot with speeds relative to the trajectory at a given time
  public void driveWithRelativeSpeeds(Trajectory trajectory, double time) {
    ChassisSpeeds speeds = controller.calculate(getPose(), trajectory.sample(time));
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);

    // Aqui você pode utilizar feedforward e PID (se precisar) para o controler de velocidade
    // Nós fizemos desse jeito mais por falta de experiência com trajetória
    tankDrive(wheelSpeeds.leftMetersPerSecond / MAX_VELOCITY, wheelSpeeds.rightMetersPerSecond / MAX_VELOCITY);
  }

  // drives the robot with tank drive
  public void tankDrive(double leftSpeed, double rightSpeed) {
    diffDrive.tankDrive(leftSpeed, rightSpeed);
  }

  // returns the current pose of the robot
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  @Override
  public void periodic() {
    odometry.update(gyro.getRotation2d(), leftMaster.getEncoder().getPosition(), rightMaster.getEncoder().getPosition());
  }
}
