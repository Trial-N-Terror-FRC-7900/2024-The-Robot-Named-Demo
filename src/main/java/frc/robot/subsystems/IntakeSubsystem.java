package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  //private static final int kCanID = 9;
  //private static final MotorType kMotorType = MotorType.kBrushless;

  public CANSparkMax m_intakeMotor = new CANSparkMax(IntakeConstants.IntakeMotorCanID, CANSparkLowLevel.MotorType.kBrushless);

  public IntakeSubsystem(){
    m_intakeMotor.restoreFactoryDefaults();
  }
    //m_intakeMotor.restoreFactoryDefaults();
    //this.setDefaultCommand(intakeOffCommand());
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command intakeOnCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem. 
    //m_intakeMotor.set(IntakeConstants.IntakeSpeed);
    return this.runOnce(() -> m_intakeMotor.set(IntakeConstants.IntakeSpeed));
  }

  public Command intakeOffCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    //m_intakeMotor.set(0);
    return this.runOnce(() -> m_intakeMotor.set(0));
  }

  public Command IntakeSpit(){
    //m_intakeMotor.set(-IntakeConstants.IntakeSpeed);
    return this.runOnce(() -> m_intakeMotor.set(-IntakeConstants.IntakeSpeed));
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
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