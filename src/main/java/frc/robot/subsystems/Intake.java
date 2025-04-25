// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// WPILib imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// REV imports
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkLowLevel.MotorType;

// Constants
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private SparkMax pivotMotor;
  private SparkMax intakeMotor;
  private SparkMaxConfig pivotConfig;
  private SparkMaxConfig intakeConfig;
  private SparkClosedLoopController pivotCLC;
  private SparkClosedLoopController intakeCLC;
  private RelativeEncoder pivotEncoder;
  private DigitalInput coralSensor;
  private DigitalInput algaeSensor;

  public static enum IntakeStates {
    Idle,
    IntakeAlgae,
    OuttakeAlgae,
    OuttakeCoral,
    ResetArm
  }

  private IntakeStates intakeState;
  private boolean pyramidMode;
  private Timer timer;

  public Intake() {
    // Actuators
    pivotMotor = new SparkMax(5, MotorType.kBrushless);
    intakeMotor = new SparkMax(6, MotorType.kBrushless);

    pivotCLC = pivotMotor.getClosedLoopController();
    intakeCLC = intakeMotor.getClosedLoopController();

    pivotConfig = new SparkMaxConfig();
    intakeConfig = new SparkMaxConfig();

    pivotConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(30);
    intakeConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(30);

    pivotConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.1) // TODO(landon): Tune
      .outputRange(-1,1);
    
    intakeConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.1) // TODO(landon): Tune
      .velocityFF(1.0 / 5767)
      .outputRange(-1, 1);

    pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Sensors
    coralSensor = new DigitalInput(0);
    algaeSensor = new DigitalInput(1);

    //super.setDefaultCommand(new RunCommand(() -> {requestState(States.Idle);}));
    intakeState = IntakeStates.Idle;
  }

  // Set Pivot position or voltage based on boolean manual value
  public void setPivotSetpoint(double setpoint, boolean manual) {
    if(!manual) {
      pivotCLC.setReference(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    } else {
      pivotCLC.setReference(setpoint, ControlType.kVoltage, ClosedLoopSlot.kSlot0);
    }
  }

  // Same as above
  public void setIntakeSetpoint(double setpoint, boolean manual) {
    if(!manual) {
      intakeCLC.setReference(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    } else {
      intakeCLC.setReference(setpoint, ControlType.kVoltage, ClosedLoopSlot.kSlot0);
    }
  }

  // Negate as needed
  public boolean hasCoral() {
    return coralSensor.get();
  }

  // Negate as needed
  public boolean hasAlgae() {
    return algaeSensor.get();
  }

  public void requestState(IntakeStates requestedState) {
    if(requestedState == IntakeStates.ResetArm) {
      timer.reset();
      timer.start();
    }
    // Add any safety conditions here..
    // Would likely need to break this up into different methods to ensure successful state change
    intakeState = requestedState;
  }

  public void togglePyramidMode() {
    pyramidMode = !pyramidMode;
  }

  // This state machine sends CAN frames once every scheduler run as it is called in periodic().
  // If - in the future - we have issues with performance or CAN utilization,
  // we can look into optimizing this to only send CAN frames once per state change.
  // The issue is that we have some conditions (pyramidMode and pivot position) that can affect
  // the state. We would need to handle this with some other method that runs every scheduler update.
  public void stateMachine() {
    switch(intakeState) {
      case Idle:
        setPivotSetpoint(IntakeConstants.kIntakeArmUp, false);
        setIntakeSetpoint(IntakeConstants.kIntakeRollerIdle, false);
        break;
      case IntakeAlgae:
        setPivotSetpoint(IntakeConstants.kIntakeArmDown, false);
        setIntakeSetpoint(IntakeConstants.kIntakeRollerIn, false);
        if(hasAlgae()) intakeState = IntakeStates.Idle;
        break;
      case OuttakeAlgae:
        setPivotSetpoint(IntakeConstants.kIntakeArmUp, false);
        setIntakeSetpoint(IntakeConstants.kIntakeRollerOut, false);
        break;
      case OuttakeCoral:
        if(pyramidMode) {
          setPivotSetpoint(IntakeConstants.kIntakeArmDown, false);
        } else {
          setPivotSetpoint(IntakeConstants.kIntakeArmPyramid, false);
        }
        if(pivotEncoder.getPosition() > 5) {
          setIntakeSetpoint(IntakeConstants.kIntakeRollerIn, false);
        } else {
          setIntakeSetpoint(IntakeConstants.kIntakeRollerIdle, false);
        }
        break;
      case ResetArm:
        if(timer.hasElapsed(2)) {
          pivotEncoder.setPosition(0.0);
          intakeState = IntakeStates.Idle;
          timer.stop();
        }
        setPivotSetpoint(-0.3, true);
        break;
      default:
        break;
    }
  }

  @Override
  public void periodic() {
    stateMachine();
    SmartDashboard.putBoolean("Has Algae?", hasAlgae());
    SmartDashboard.putBoolean("Has Coral?", hasCoral());
    SmartDashboard.putString("State", intakeState.toString());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
