// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private SparkMax climbMotor;
  private SparkMaxConfig climbConfig;
  private SparkClosedLoopController climbCLC;
  
  public Climb() {
    climbMotor = new SparkMax(7, MotorType.kBrushless);
    climbCLC = climbMotor.getClosedLoopController();
    climbConfig = new SparkMaxConfig();

    climbConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(30);
    
    climbConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.1)
      .outputRange(-1, 1);

    climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void climb() {
    climbCLC.setReference(ClimbConstants.kClimbSetpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void unclimb() {
    climbCLC.setReference(ClimbConstants.kIdleSetpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
