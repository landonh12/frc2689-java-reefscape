// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.DriveConstants;

public class Drive extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  
  SparkMax leftLeader;
  SparkMax leftFollower;
  SparkMax rightLeader;
  SparkMax rightFollower;

  DifferentialDrive driveController;
  DifferentialDrivePoseEstimator drivePoseEstimator;

  public Drive() {
    leftLeader = new SparkMax(1, MotorType.kBrushless);
    leftFollower = new SparkMax(2, MotorType.kBrushless);
    rightLeader = new SparkMax(3, MotorType.kBrushless);
    rightFollower = new SparkMax(4, MotorType.kBrushless);

    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

    globalConfig.smartCurrentLimit(60).idleMode(IdleMode.kBrake);
    rightLeaderConfig.apply(globalConfig).inverted(true);
    leftFollowerConfig.apply(globalConfig).follow(leftLeader);
    rightFollowerConfig.apply(globalConfig).follow(rightLeader);

    leftLeader.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveController = new DifferentialDrive(leftLeader, rightLeader);
  }

  public void driveOpenLoop(double speed, double rotation) {
    driveController.arcadeDrive(speed, rotation * DriveConstants.kRotationFactor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber('leftSpeed', leftLeader.getEncoder())
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}