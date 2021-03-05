// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class DriveTrain extends SubsystemBase {
  private final WPI_TalonFX rightMasterMotor;
  private final WPI_TalonFX leftMasterMotor;
  private final WPI_TalonSRX PigeonController;

  private final WPI_TalonFX rightSlaveMotor;
  private final WPI_TalonFX leftSlaveMotor;

  private PigeonIMU pigeon;

  private SpeedControllerGroup left_falcons;
  private SpeedControllerGroup right_falcons;

  private final DifferentialDrive m_drive;

  private DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    leftMasterMotor = new WPI_TalonFX(DriveConstants.kLeftMasterPort);
    leftSlaveMotor = new WPI_TalonFX(DriveConstants.kLeftSlave0Port);
    PigeonController = new WPI_TalonSRX(DriveConstants.kPigeonPort);

    rightMasterMotor = new WPI_TalonFX(DriveConstants.kRightMasterPort);
    rightSlaveMotor = new WPI_TalonFX(DriveConstants.kRightSlave0Port);

    pigeon = new PigeonIMU(PigeonController);

  //Set Electronics To Default
    leftMasterMotor.configFactoryDefault();
    leftSlaveMotor.configFactoryDefault();
    rightMasterMotor.configFactoryDefault();
    rightSlaveMotor.configFactoryDefault();

    leftMasterMotor.setNeutralMode(NeutralMode.Coast);
    leftSlaveMotor.setNeutralMode(NeutralMode.Coast);
    rightMasterMotor.setNeutralMode(NeutralMode.Coast);
    rightSlaveMotor.setNeutralMode(NeutralMode.Coast);

    leftMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    left_falcons = new SpeedControllerGroup(leftMasterMotor, leftSlaveMotor);
    right_falcons = new SpeedControllerGroup(rightMasterMotor, rightSlaveMotor);

    left_falcons.setInverted(true);
    right_falcons.setInverted(true);

    m_drive = new DifferentialDrive(left_falcons, right_falcons);

    m_odometry = new DifferentialDriveOdometry(new Rotation2d(0));

    zeroSensors();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(
      Rotation2d.fromDegrees(getDirection()),
      getLeftDistance(),
      getRightDistance()
    );

    SmartDashboard.putNumber("Left Encoder", getLeftDistance());
    SmartDashboard.putNumber("Right Encoder", getRightDistance());
    SmartDashboard.putNumber("Heading", getDirection());

  }

  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    left_falcons.setVoltage(leftVolts);
    right_falcons.setVoltage(-rightVolts);
    m_drive.feed();
  }  

  public void resetEncoders() {
    leftMasterMotor.setSelectedSensorPosition(0);
    rightMasterMotor.setSelectedSensorPosition(0);
  }

  public void resetDirection() {
    pigeon.setFusedHeading(0);
  }

  public void zeroSensors() {
    resetEncoders();
    resetDirection();
    m_odometry.resetPosition(
      new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)),
      Rotation2d.fromDegrees(0)
    );

    System.out.println(getDirection());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
  }

  public double getDirection() {
    return Math.IEEEremainder(pigeon.getFusedHeading(), 360);
  }

  public double getLeftVoltage() {
    return leftMasterMotor.getMotorOutputVoltage();
  }

  public double getRightVoltage() {
    return rightMasterMotor.getMotorOutputVoltage();
  }

  public double getAverageDistance() {
    return (getLeftDistance() + getRightDistance()) / 2;
  }

  // distance in meters
  public double getLeftDistance() {
    return -1 * leftMasterMotor.getSelectedSensorPosition() * DriveConstants.kWheelDistancePerPulse;
  }

  // velocity in meters / sec
  public double getLeftVelocity() {
    return -1 * leftMasterMotor.getSelectedSensorVelocity() * DriveConstants.kWheelDistancePerPulse;
  }

  public double getRightDistance() {
    return rightMasterMotor.getSelectedSensorPosition() * DriveConstants.kWheelDistancePerPulse;
  }

  public double getRightVelocity() {
    return rightMasterMotor.getSelectedSensorVelocity() * DriveConstants.kWheelDistancePerPulse;
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

}
