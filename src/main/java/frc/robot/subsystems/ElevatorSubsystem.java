package frc.robot.subsystems;

//import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj.Servo;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private CANSparkMax m_elevatorMotor1;
  private CANSparkMax m_elevatorMotor2;
  private SparkPIDController m_pidController1;
  private SparkPIDController m_pidController2;
  private Servo m_greenServo;
  private Servo m_purpleServo;

  private boolean homeinginProgress1 = false;
  private boolean homeinginProgress2 = false;

  private int invertBoolean =1;

  public ElevatorSubsystem() {

    m_elevatorMotor1 = new CANSparkMax(ElevatorConstants.ElevatorMotor1CanID, CANSparkLowLevel.MotorType.kBrushless);
    m_elevatorMotor1.restoreFactoryDefaults();
    m_elevatorMotor1.setIdleMode(IdleMode.kBrake);

    m_elevatorMotor2 = new CANSparkMax(ElevatorConstants.ElevatorMotor2CanID, CANSparkLowLevel.MotorType.kBrushless);
    m_elevatorMotor2.restoreFactoryDefaults();
    m_elevatorMotor2.setIdleMode(IdleMode.kBrake);
    m_greenServo = new Servo(ElevatorConstants.servoIDGreen);
    m_purpleServo = new Servo(ElevatorConstants.servoIDPurple);

  /**
   * In order to use PID functionality for a controller, a SparkPIDController object
   * is constructed by calling the getPIDController() method on an existing
   * CANSparkMax object
   */
  m_pidController1 = m_elevatorMotor1.getPIDController();
  m_pidController2 = m_elevatorMotor1.getPIDController();

  // set PID coefficients
  m_pidController1.setP(ElevatorConstants.kP);
  m_pidController1.setI(ElevatorConstants.kI);
  m_pidController1.setD(ElevatorConstants.kD);
  m_pidController1.setIZone(ElevatorConstants.kIz);
  m_pidController1.setFF(ElevatorConstants.kFF);
  m_pidController1.setOutputRange(ElevatorConstants.kMinOutput, ElevatorConstants.kMaxOutput);
  
  m_pidController2.setP(ElevatorConstants.kP);
  m_pidController2.setI(ElevatorConstants.kI);
  m_pidController2.setD(ElevatorConstants.kD);
  m_pidController2.setIZone(ElevatorConstants.kIz);
  m_pidController2.setFF(ElevatorConstants.kFF);
  m_pidController2.setOutputRange(ElevatorConstants.kMinOutput, ElevatorConstants.kMaxOutput);



  m_elevatorMotor1.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ElevatorConstants.elevatorForwardLimit);
  m_elevatorMotor1.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ElevatorConstants.elevatorReverseLimit);

  //m_ElevatorMotor1.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
  //m_ElevatorMotor1.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

  m_elevatorMotor2.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ElevatorConstants.elevatorForwardLimit);
  m_elevatorMotor2.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ElevatorConstants.elevatorReverseLimit);

  //m_ElevatorMotor2.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
  //m_ElevatorMotor2.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
  }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command elevatorMotorMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public Command boolOn(){
  return this.run(()-> {
    invertBoolean = 1;
  });
}

public Command boolOff(){
  return this.run(()-> {
    invertBoolean = -1;
  });
}

public Command elevatorUp() {
    return this.run(() -> {
      m_elevatorMotor1.set(-0.8);
      m_elevatorMotor2.set(-0.8*invertBoolean);
    });

}

public Command elevatorDown() {
    return this.run(() -> {
      m_elevatorMotor1.set(0.8);
      m_elevatorMotor2.set(0.8*invertBoolean);
    });
}

public Command elevatorOff() {
    return this.run(() -> {
      m_elevatorMotor1.set(0);
      m_elevatorMotor2.set(0);
    });
}

public Command elevatorHome(){
  return this.run(() -> {
      homeinginProgress1 = true;
      homeinginProgress2 = true;
      m_elevatorMotor1.set(-0.1);
      m_elevatorMotor2.set(-0.1);
  });
}

public Command elevatorUnlock(){
  return this.run(()-> {
      m_greenServo.setAngle(ElevatorConstants.greenUnlockDegrees);
      m_purpleServo.setAngle(ElevatorConstants.purpleUnlockDegrees);
  });
}

public Command elevatorLock(){
  return this.run(()-> {
      m_greenServo.setAngle(ElevatorConstants.greenLockDegrees);
      m_purpleServo.setAngle(ElevatorConstants.purpleLockDegrees);
  });
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
    if(homeinginProgress1){
      if(m_elevatorMotor1.getOutputCurrent() >= ElevatorConstants.homingCurrent){
        m_elevatorMotor1.getEncoder().setPosition(0);
        m_elevatorMotor1.set(0);
      }
    }
    if(homeinginProgress2){
      if(m_elevatorMotor2.getOutputCurrent() >= ElevatorConstants.homingCurrent){
        m_elevatorMotor2.getEncoder().setPosition(0);
        m_elevatorMotor2.set(0);
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
