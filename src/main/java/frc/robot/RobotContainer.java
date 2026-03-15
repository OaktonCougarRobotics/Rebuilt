// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.Vision;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public RobotState robotState = RobotState.NEUTRAL;
  public final Drivetrain m_drivetrain;
  public final Vision m_vision;
  public final Shooter m_shooter;
  // public final Shooter m_shooter;
  // public final ShootCommand m_shootCommand;
  Intake m_intake = new Intake(30, 52, () -> robotState);
    private final Joystick m_joystick = new Joystick(1);
  // public Jank jank = new Jank(0,      () -> m_joystick.getRawButtonPressed(2),
  //     () -> m_joystick.getRawButtonPressed(1));
  public final Joystick m_buttonboardA = new Joystick(0);
  public final Joystick m_buttonboardB = new Joystick(2);
  public final Trigger visionOnChanger = new Trigger(() -> m_buttonboardB.getRawButton(2));
  final Command driveCommand;
  IntakeCommand intakeCommand = new IntakeCommand(
    m_intake, 
    ()-> false, 
    () -> (m_buttonboardA.getRawButton(15)?RobotState.DEFENSE:(m_buttonboardA.getRawButton(16)?RobotState.INTAKE:RobotState.NEUTRAL)),//top 21 and bottom 22 down
    () -> {if(m_buttonboardA.getRawButton(13)) return RobotState.INTAKE; else if (m_buttonboardA.getRawButton(14)) return RobotState.OUTTAKE; else return RobotState.NEUTRAL;},    
    () -> {if(m_buttonboardA.getRawButton(11)) return -1.0; else if (m_buttonboardA.getRawButton(12)) return 1.0; else return 0.0;},
    0.005,
    0.0,
    0.0
  );
  private final Trigger navxResetButton = new Trigger(() -> m_joystick.getRawButton(3));
  private final Trigger trenchLockButton = new Trigger(() -> m_joystick.getRawButton(4));
  private final Trigger swerveLockButton = new Trigger(() -> m_joystick.getRawButton(1));
  public boolean isTrenchLock;
  private final Trigger alignTrigger = new Trigger(() -> m_joystick.getRawButton(6));
  private final Trigger flywheelTrigger = new Trigger(() -> m_buttonboardA.getRawButton(6));
  private final Trigger indexTrigger = new Trigger(() -> m_buttonboardA.getRawButton(5));
  // private final Trigger left = new Trigger(() -> m_joystick.getRawButton(1));
  // private final Trigger right = new Trigger(() -> m_joystick.getRawButton(2));

  private SendableChooser<Command> autoChooser;
  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    m_shooter = new Shooter(58, 36);
    m_buttonboardA.getRawButton(21);
    m_drivetrain = new Drivetrain(
      new File(Filesystem.getDeployDirectory(), "swerve"));
      m_vision = new Vision(m_drivetrain.swerveDrive::addVisionMeasurement, m_drivetrain);
      // m_shooter = new Shooter(m_drivetrain, 0, 0);
      
    driveCommand = new DriveCommand(
      m_drivetrain,
      () -> robotState,
      () -> -m_joystick.getRawAxis(1) * (DriverStation.getAlliance().get()==Alliance.Blue && m_vision.visionOn?1:-1),
      () -> -m_joystick.getRawAxis(0) * (DriverStation.getAlliance().get()==Alliance.Blue && m_vision.visionOn?1:-1),
      () -> m_joystick.getRawAxis(2) * -1,
      () -> isTrenchLock,
      0.072,
      0.0,
      0.0);
    // Configure the trigger bindings
    autoChooser = AutoBuilder.buildAutoChooser();
    // m_shootCommand = new ShootCommand(m_shooter,()-> m_buttonboardA.getRawButtonPressed(4), ()->m_buttonboardA.getRawButton(5), ()->0.0);
    SmartDashboard.putData("Auto Chooser", autoChooser);

    NamedCommands.registerCommand("Shoot", new ShootCommand(m_shooter, ()->true, ()-> false, ()-> 0.0, ()->false));
    // sysRoutine = m_drivetrain.getSysIdCommand();
        configureBindings();
  }

  /** 
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_drivetrain.setDefaultCommand(driveCommand);
    m_intake.setDefaultCommand(intakeCommand);
    
    // jan  
    // m_shooter.setDefaultCommand(m_shootCommand);
    visionOnChanger.onTrue(Commands.runOnce(()-> {m_vision.visionOn=false;}, m_vision));
    visionOnChanger.onFalse(Commands.runOnce(()-> {m_vision.visionOn=true;}, m_vision));
    navxResetButton.onTrue(Commands.runOnce(m_drivetrain::resetEverything));
    alignTrigger.whileTrue(Commands.runOnce(() -> robotState = RobotState.SHOOT));
    alignTrigger.whileFalse(Commands.runOnce(() -> robotState = RobotState.NEUTRAL));
    trenchLockButton.onTrue(Commands.runOnce(() ->{isTrenchLock = true;}, m_drivetrain));
    trenchLockButton.onFalse(Commands.runOnce(() ->{isTrenchLock = false;}, m_drivetrain ));
    flywheelTrigger.whileTrue(Commands.run(() -> m_shooter.shooterMotor.setVoltage(Constants.MAX_FLYWHEEL_VOLTAGE)));
    flywheelTrigger.whileFalse(Commands.run(() -> m_shooter.shooterMotor.setVoltage(0)));
    indexTrigger.whileTrue(Commands.run(() -> m_shooter.indexMotor.setVoltage(Constants.MAX_INDEX_VOLTAGE)));
    indexTrigger.whileFalse(Commands.run(() -> m_shooter.indexMotor.setVoltage(0)));

    swerveLockButton.whileTrue(Commands.run(() -> m_drivetrain.swerveDrive.lockPose()));
    swerveLockButton.whileFalse(Commands.run(() -> driveCommand.execute(), m_drivetrain));

    //FINISH THIS

    // (new Trigger(()->m_buttonboardA.getRawButton(4))).whileTrue(new ShootCommand(m_shooter,()-> true, ()->false, ()->0.0, ()->true));
// m_shooter,()-> m_buttonboardA.getRawButtonPressed(4), ()->m_buttonboardA.getRawButton(5), ()->0.0
    // left.onTrue(Commands.runOnce(()->{m_outtake.runIndex(2.0); m_outtake.runShooter(2.0);}));
    // right.onTrue(m_outtake.runShooter(6.0));
    // left.onFalse(Commands.runOnce(()->{m_outtake.runIndex(0.0); m_outtake.runShooter(0.0);}));    // right.onFalse(Commands.runOnce(()->m_outtake.runShooter(0.0), m_outtake));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // String name = autoChooser.getSelected().getName();
    // System.out.println(name);
    //return new PathPlannerAuto("Rightest Auto");
    String x = autoChooser.getSelected().getName();
    return new PathPlannerAuto(x);
  }

  public void resetPose(){
    m_drivetrain.swerveDrive.resetOdometry(new Pose2d());
  }
  
  public void periodic(){
    // System.out.println(m_drivetrain.swerveDrive.getPose());
    // System.out.println(m_drivetrain.orientationError());
    // System.out.println("Angle: " + m_drivetrain.hubAngle());
    // System.out.println("X: " + m_drivetrain.swerveDrive.getPose().getX());
    // System.out.println("Y: " + m_drivetrain.swerveDrive.getPose().getY());
    
    // System.out.println(m_drivetrain.getHeadingError());
    // SmartDashboard.putNumber("rotation", m_shooter.getOrientationError());
    // SmartDashboard.putNumber("tangential", m_shooter.getTangentialVelocity());
    // SmartDashboard.putNumber("radial", m_shooter.getRadialVelocity());
    SmartDashboard.putNumber("Velocity", Math.sqrt(Math.pow(m_drivetrain.swerveDrive.getFieldVelocity().vxMetersPerSecond,2)+Math.pow(m_drivetrain.swerveDrive.getFieldVelocity().vyMetersPerSecond,2)));
    SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    SmartDashboard.putNumber("intake wrist", m_intake.intakeMotor.getPosition().getValueAsDouble());
  }

  public enum RobotState{
    NEUTRAL,
    INTAKE,
    SHOOT,
    OUTTAKE,
    DEFENSE
  }
}
