package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  //private static final MotorType kMotorType = MotorType.kBrushless;
  //private int flip = -1;

  private CANSparkMax m_armMotor1;
  private CANSparkMax m_armMotor2; 
  private SparkPIDController m_pidController;
  private AbsoluteEncoder m_encoder;
  

  public ArmSubsystem() {

    //Right from front arm motor
    m_armMotor1 = new CANSparkMax(ArmConstants.armMotor1CanID, CANSparkLowLevel.MotorType.kBrushless);
    m_armMotor1.restoreFactoryDefaults();
    m_armMotor1.setSmartCurrentLimit(30);

    //Left from front arm motor
    m_armMotor2 = new CANSparkMax(ArmConstants.armMotor2CanID, CANSparkLowLevel.MotorType.kBrushless);
    m_armMotor2.restoreFactoryDefaults();
    m_armMotor2.setSmartCurrentLimit(30);

    m_encoder = m_armMotor2.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    m_encoder.setZeroOffset(ArmConstants.EncoderZeroPosOffset);

    /**
     * From here on out, code looks exactly like running PID control with the 
     * built-in NEO encoder, but feedback will come from the alternate encoder
     */

    /**
     * In order to use PID functionality for a controller, a SparkPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_pidController = m_armMotor2.getPIDController();

    // Encoder object created to display position values
    //m_encoder = m_armMotor1.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    // set PID coefficients
    m_pidController.setP(ArmConstants.kP);
    m_pidController.setI(ArmConstants.kI);
    m_pidController.setD(ArmConstants.kD);
    m_pidController.setIZone(ArmConstants.kIz);
    m_pidController.setFF(ArmConstants.kFF);
    m_pidController.setOutputRange(ArmConstants.kMinOutput, ArmConstants.kMaxOutput);
    m_pidController.setFeedbackDevice(m_encoder);


    m_encoder.setPositionConversionFactor(360);

    m_armMotor2.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ArmConstants.armForwardLimit);
    m_armMotor2.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ArmConstants.armReverseLimit);
    //negative motor movement = arm up

    m_armMotor2.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    m_armMotor2.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    m_armMotor1.follow(m_armMotor2, true);
  }

  /*
  public void placeholder(CANSparkMax motorset1, CANSparkMax motorset2, double value){
    m_placeholder.set(value);
    motorset1.set(m_placeholder.get());
    motorset2.set(m_placeholder.get()*flip);
    return;
  }
  */

  public Command armLowReady(){
    return this.runOnce(() -> m_pidController.setReference(ArmConstants.armDown, CANSparkMax.ControlType.kPosition));
  }
  public Command armAmp(){
    //call to specific rotation first
    //m_armMotor1.set(0);
    return this.run(() -> m_pidController.setReference(ArmConstants.armAmp, CANSparkMax.ControlType.kPosition));
  }

  public Command armSpeaker() {
    //m_armMotor1.set(1*flip);
    //m_armMotor2.set(-1*flip);
    //m_armMotor1.get();
    return this.runOnce(() -> m_pidController.setReference(ArmConstants.armSpeaker, CANSparkMax.ControlType.kPosition));
}

/*
  public Command armAmp(){
    return null;
  }

  public Command armSpeaker(){
    return null;
  }

  //public List<List> i = [];
  public Command clickUp(){
    return null;
  }
  */

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
    SmartDashboard.putNumber("encoder position:", m_encoder.getPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}