// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.swing.GroupLayout.SequentialGroup;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));

  // A chooser for autonomous commands
  private final SendableChooser<Command> autoChooser;

  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController opperatorXbox = new CommandXboxController(2);

  public double invertJoy = 1;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
     //Register Named Commands
      //Intake Commands
     NamedCommands.registerCommand("IntakeON", m_intake.intakeOnCommand());       
       NamedCommands.registerCommand("ShooterIntakeON", m_shooter.Intake());
     NamedCommands.registerCommand("IntakeOFF", m_intake.intakeOffCommand());
       NamedCommands.registerCommand("ShooterIntakeOFF", m_shooter.shootOff());
      //Shooter Commands
     NamedCommands.registerCommand("ShooterON", m_shooter.shootOn());            
     NamedCommands.registerCommand("ShooterOFF", m_shooter.shootOff());
     NamedCommands.registerCommand("ShooterSPIN", m_shooter.spinUp());
      //Arm Commands
     NamedCommands.registerCommand("ArmAMP", m_arm.armAmp());   
     NamedCommands.registerCommand("ArmSPEAKER", m_arm.armSpeaker());
     NamedCommands.registerCommand("ArmDOWN", m_arm.armLowReady());       


    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure the trigger bindings
    configureBindings();
    
    if(DriverStation.getAlliance().isPresent()){
      if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
        invertJoy = 1;
      }
      else{
        invertJoy = -1;
      }
    }
    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                          () -> invertJoy*MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                      OperatorConstants.LEFT_Y_DEADBAND),
                                                          () -> invertJoy*MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                      OperatorConstants.LEFT_X_DEADBAND),
                                                          () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                      OperatorConstants.RIGHT_X_DEADBAND),
                                                          driverXbox.getHID()::getYButtonPressed,
                                                          driverXbox.getHID()::getAButtonPressed,
                                                          driverXbox.getHID()::getXButtonPressed,
                                                          driverXbox.getHID()::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX(),
        () -> driverXbox.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX() * 0.5);

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? closedAbsoluteDriveAdv : closedAbsoluteDriveAdv);
  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    //driverXbox.button(6).onTrue((Commands.runOnce(drivebase::resetOdometry(Pose2d(0,0,0)))));
    driverXbox.button(7).onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    driverXbox.b().whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
                                   new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              ));
                              
    //use y for shooter intake (reverse shooter)
    opperatorXbox.y().onTrue(m_shooter.Intake()).onFalse(m_shooter.shootOff());

    //use a to push note to shooter
    opperatorXbox.a().onTrue(m_shooter.shootOn()).onFalse(m_shooter.shootOff());

    //use x to spin up shooter
    opperatorXbox.x().onTrue(m_shooter.spinUp()).onFalse(m_shooter.shootOff());

    //use the bumpers for amp (left) and speaker (right) positions
    opperatorXbox.leftBumper().onTrue(m_arm.armAmp());
    opperatorXbox.rightBumper().onTrue(m_arm.armSpeaker());

    //use b to return arm back to robot and use ground intake
    opperatorXbox.b().onTrue(m_arm.armLowReady());
    opperatorXbox.b().onTrue(m_intake.intakeOnCommand()).onFalse(m_intake.intakeOffCommand());
    opperatorXbox.b().onTrue(m_shooter.Intake()).onFalse(m_shooter.shootOff());

    //use Dpad up and down to control climbers
    opperatorXbox.povUp().onTrue(m_elevator.elevatorUp()).onFalse(m_elevator.elevatorOff());
    opperatorXbox.povDown().onTrue(m_elevator.elevatorDown()).onFalse(m_elevator.elevatorOff());

    opperatorXbox.povLeft().onTrue(m_elevator.elevatorLock());
    opperatorXbox.povRight().onTrue(m_elevator.elevatorUnlock());

    opperatorXbox.button(6).onTrue(m_elevator.boolOn());
    opperatorXbox.button(7).onTrue(m_elevator.boolOff());
    //opperatorXbox.setDefaultCommand();

    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
