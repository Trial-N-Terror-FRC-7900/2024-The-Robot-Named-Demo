package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
//import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
//import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  //private static final MotorType kMotorType = MotorType.kBrushless;

  private CANSparkMax m_shooterMotorTop;
  private CANSparkMax m_shooterMotorBottom;
  private CANSparkMax m_shooterMotorAmp;
  private SparkPIDController m_pidController;
  private SparkPIDController m_pidController2;
  private SparkPIDController m_pidController3;
  //private AbsoluteEncoder m_encoder;

  public ShooterSubsystem() {

    //Top shooterMotor motor
    m_shooterMotorTop = new CANSparkMax(ShooterConstants.shooterMotorTopCanID, CANSparkLowLevel.MotorType.kBrushless);
    m_shooterMotorTop.restoreFactoryDefaults();

    //Bottom shooterMotor motor
    m_shooterMotorBottom = new CANSparkMax(ShooterConstants.shooterMotorBottomCanID, CANSparkLowLevel.MotorType.kBrushless);
    m_shooterMotorBottom.restoreFactoryDefaults();

    m_shooterMotorAmp = new CANSparkMax(ShooterConstants.shooterMotorAmpCanID, CANSparkLowLevel.MotorType.kBrushless);
    m_shooterMotorAmp.restoreFactoryDefaults();


    /**
     * From here on out, code looks exactly like running PID control with the 
     * built-in NEO encoder, but feedback will come from the alternate encoder
     */ 

    // PID coefficients
    m_pidController = m_shooterMotorTop.getPIDController();
    m_pidController2 = m_shooterMotorBottom.getPIDController();
    m_pidController3 = m_shooterMotorAmp.getPIDController();

    // set PID coefficients
    m_pidController.setP(ShooterConstants.kP);
    m_pidController.setI(ShooterConstants.kI);
    m_pidController.setD(ShooterConstants.kD);
    m_pidController.setIZone(ShooterConstants.kIz);
    m_pidController.setFF(ShooterConstants.kFF);
    //m_pidController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);
    
    m_pidController2.setP(ShooterConstants.kP);
    m_pidController2.setI(ShooterConstants.kI);
    m_pidController2.setD(ShooterConstants.kD);
    m_pidController2.setIZone(ShooterConstants.kIz);
    m_pidController2.setFF(ShooterConstants.kFF);
    //m_pidController2.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);

    m_pidController3.setP(ShooterConstants.kP);
    m_pidController3.setI(ShooterConstants.kI);
    m_pidController3.setD(ShooterConstants.kD);
    m_pidController3.setIZone(ShooterConstants.kIz);
    m_pidController3.setFF(ShooterConstants.kFF);
    //m_pidController3.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);
    //m_pidController.setFeedbackDevice(m_encoder);
  }

  public Command shootOn(){
    return this.run(() -> {
      /*
      m_shooterMotorTop.set(-ShooterConstants.shootSpeed);
      m_shooterMotorBottom.set(ShooterConstants.shootSpeed);
      */
      m_pidController3.setReference(ShooterConstants.shootSpeed, CANSparkMax.ControlType.kVelocity);
    });
  }
  public Command shootOff(){
    return this.run(() -> {
      m_shooterMotorTop.set(ShooterConstants.noShot);
      m_shooterMotorBottom.set(ShooterConstants.noShot);
      m_shooterMotorAmp.set(ShooterConstants.noShot);
    });
  }

  public Command spinUp(){
    return this.run(()->{
      m_pidController.setReference(-ShooterConstants.shootSpeed, CANSparkMax.ControlType.kVelocity);
      m_pidController2.setReference(ShooterConstants.shootSpeed, CANSparkMax.ControlType.kVelocity);
    });
  }

  public Command Intake(){
    return this.run(() -> {
      m_shooterMotorTop.set(0.25);
      m_shooterMotorBottom.set(-0.25);
      m_shooterMotorAmp.set(0.25);
    });
  }

  public Command AmpShotOn(){
    return this.run(()-> {
      m_pidController.setReference(ShooterConstants.AmpShotSpeed, CANSparkMax.ControlType.kVelocity);
      m_pidController2.setReference(-ShooterConstants.AmpShotSpeed, CANSparkMax.ControlType.kVelocity);
      m_pidController3.setReference(-ShooterConstants.shootSpeed, CANSparkMax.ControlType.kVelocity);
    });
  }
/*
  public Command AmpShotOff(){
    return this.run(()-> {
      m_shooterMotorTop.set(ShooterConstants.noShot);
      m_shooterMotorBottom.set(ShooterConstants.noShot);
      m_shooterMotorAmp.set(ShooterConstants.noShot);
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
    SmartDashboard.putNumber("Shooter Top Speed", m_shooterMotorTop.getEncoder().getVelocity());
    SmartDashboard.putNumber("Shooter Bottom Speed", m_shooterMotorBottom.getEncoder().getVelocity());
    SmartDashboard.putNumber("Shooter Top Current", m_shooterMotorTop.getOutputCurrent());
    SmartDashboard.putNumber("Shooter Bottom Current", m_shooterMotorBottom.getOutputCurrent());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}