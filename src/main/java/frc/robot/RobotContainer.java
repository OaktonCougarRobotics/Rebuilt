// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.Vision;

import java.io.File;
import java.util.function.DoubleSupplier;


import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  public final Intake m_intake = new Intake(30, 52, () -> robotState);
  // The robots joystick and buttonboards are defined here...
  public final Joystick m_joystick = new Joystick(1);
  public final Joystick m_buttonboardA = new Joystick(0);
  public final Joystick m_buttonboardB = new Joystick(2);
  // The different buttons on the joystick and buttonboards are defined here...

    // Magic Buttons First:
    public final Trigger swerveLockButton = new Trigger(() -> m_joystick.getRawButton(1));
    public final Trigger alignTrigger = new Trigger(() -> m_joystick.getRawButton(6));
    public final Trigger trenchLockButton = new Trigger(() -> m_joystick.getRawButton(4));
    public final Trigger intakeUp = new Trigger(() -> m_buttonboardA.getRawButton(15));
    public final Trigger intakeDown = new Trigger(() -> m_buttonboardA.getRawButton(16));
    public final Trigger shooterTrigger = new Trigger(() ->m_buttonboardB.getRawButton(8));
    public final Trigger alignIntakeTrigger = new Trigger(() -> m_buttonboardA.getRawButton(2));
    // Manuals/Overrides here  
      // Manual Intake
      public final Trigger intakeNuke = new Trigger(()->m_buttonboardB.getRawButton(1));
      public final Trigger manualIntakeMotorUp = new Trigger(() -> m_buttonboardA.getRawButton(11));
      public final Trigger manualIntakeMotorDown = new Trigger(() -> m_buttonboardA.getRawButton(12));
      public final Trigger manualFeederMotorIn = new Trigger(() -> m_buttonboardA.getRawButton(10));
      public final Trigger manualFeederMotorOut = new Trigger(() -> m_buttonboardA.getRawButton(9));

    public final Trigger visionOnChanger = new Trigger(() -> m_buttonboardB.getRawButton(2));
    public final Trigger navxResetButton = new Trigger(() -> m_joystick.getRawButton(3));
      // Manual Shooter
      public final Trigger shooterNuke = new Trigger(() -> m_buttonboardA.getRawButton(1));
      public final DoubleSupplier flywheelDial = () -> m_buttonboardB.getRawAxis(6);
      public final Trigger indexTrigger = new Trigger(() -> m_buttonboardA.getRawButton(5));

  
  public boolean isTrenchLock;
  public boolean isShooterManual;
  
  // private final Trigger left = new Trigger(() -> m_joystick.getRawButton(1));
  // private final Trigger right = new Trigger(() -> m_joystick.getRawButton(2));
  public final Command driveCommand;
  private final Command shootCommand;
  double joystickDegree = 2.0;
  private SendableChooser<Command> autoChooser;
  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putNumber("Joystick Degree", 2.0);
    m_shooter = new Shooter(58, 36);
    m_buttonboardA.getRawButton(21);
    m_drivetrain = new Drivetrain(
      new File(Filesystem.getDeployDirectory(), "swerve"));
      m_vision = new Vision(m_drivetrain.swerveDrive::addVisionMeasurement, m_drivetrain);
      // m_shooter = new Shooter(m_drivetrain, 0, 0);
    driveCommand = new DriveCommand(
      m_drivetrain,
      () -> robotState,
      () -> { boolean isRed = DriverStation.getAlliance().get()==Alliance.Red ;SmartDashboard.putBoolean("isRed", isRed); return -1 * Math.abs(Math.pow(m_joystick.getRawAxis(1), joystickDegree)) * Math.signum(m_joystick.getRawAxis(1)) * (isRed && m_vision.visionOn?-1:1);},
      () -> { boolean isRed = DriverStation.getAlliance().get()==Alliance.Red ;SmartDashboard.putBoolean("isRed", isRed); return -1 * Math.abs(Math.pow(m_joystick.getRawAxis(0), joystickDegree)) * Math.signum(m_joystick.getRawAxis(0)) * (isRed && m_vision.visionOn?-1:1);},
      // () -> { boolean isRed = DriverStation.getAlliance().get()==Alliance.Red ;SmartDashboard.putBoolean("isRed", isRed); return -1 * m_joystick.getRawAxis(1) * (isRed && m_vision.visionOn?-1:1);},
      // () -> { boolean isRed = DriverStation.getAlliance().get()==Alliance.Red ;SmartDashboard.putBoolean("isRed", isRed); return -1 * m_joystick.getRawAxis(0) * (isRed && m_vision.visionOn?-1:1);},
      () -> -1 * m_joystick.getRawAxis(2),
      () -> isTrenchLock,
      0.072,
      0.0,
      0.0);
    shootCommand = new ParallelCommandGroup (
                Commands.run(() -> m_shooter.shooterMotor.setVoltage(Constants.MAX_FLYWHEEL_VOLTAGE)),
                new SequentialCommandGroup(
                    Commands.waitSeconds(1.5),//  TEST TS
                    Commands.run(() -> m_shooter.indexMotor.setVoltage(Constants.MAX_INDEX_VOLTAGE))
                )
            ).finallyDo((x)->{m_shooter.shooterMotor.set(0); m_shooter.indexMotor.set(0);});
    shootCommand.addRequirements(m_shooter);
    // Configure the trigger bindings
    autoChooser = AutoBuilder.buildAutoChooser();
    // m_shootCommand = new ShootCommand(m_shooter,()-> m_buttonboardA.getRawButtonPressed(4), ()->m_buttonboardA.getRawButton(5), ()->0.0);
    SmartDashboard.putData("Auto Chooser", autoChooser);

    NamedCommands.registerCommand("Shoot", shootCommand);
    // // sysRoutine = m_drivetrain.getSysIdCommand();
        configureBindings();

    NamedCommands.registerCommand("Intake", Commands.run(()->{                
      m_intake.intakeMotor.setControl(new MotionMagicDutyCycle(Constants.INTAKE_DOWN_POSITION));
      m_intake.feederWheel.set((Math.abs(m_intake.intakeMotor.getPosition().getValueAsDouble()-Constants.INTAKE_DOWN_POSITION)<.7?Constants.MAX_FLYWHEEL_VOLTAGE:0));
      }, m_intake));
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
    // Vision enable/disable
    visionOnChanger.onTrue(Commands.runOnce(()-> {m_vision.visionOn=false;}, m_vision));
    visionOnChanger.onFalse(Commands.runOnce(()-> {m_vision.visionOn=true;}, m_vision));
    // Navx reset
    navxResetButton.onTrue(Commands.runOnce(m_drivetrain::resetEverything));
    // Auto-Lock
    alignTrigger.whileTrue(Commands.runOnce(() -> robotState = RobotState.SHOOT));
    alignTrigger.whileFalse(Commands.runOnce(() -> robotState = RobotState.NEUTRAL));
    // Trench-Lock
    trenchLockButton.onTrue(Commands.runOnce(() ->{isTrenchLock = true;}, m_drivetrain));
    trenchLockButton.onFalse(Commands.runOnce(() ->{isTrenchLock = false;}, m_drivetrain ));
    // 
    // flywheelTrigger.whileTrue(Commands.run(() -> m_shooter.shooterMotor.setVoltage(Constants.MAX_FLYWHEEL_VOLTAGE)));
    // flywheelTrigger.whileFalse(Commands.run(() -> m_shooter.shooterMotor.setVoltage(0)));
    indexTrigger.whileTrue(Commands.run(() -> m_shooter.indexMotor.setVoltage(Constants.MAX_INDEX_VOLTAGE)));
    indexTrigger.whileFalse(Commands.run(() -> m_shooter.indexMotor.setVoltage(0)));

    shooterNuke.onTrue(Commands.runOnce(()->{isShooterManual = true;}));
    shooterNuke.onFalse(Commands.runOnce(()->{isShooterManual = false;}));

    intakeUp.whileTrue(Commands.run(()->{                
      m_intake.intakeMotor.setControl(new MotionMagicDutyCycle(0));
      m_intake.feederWheel.set(0);
      }, m_intake));
    intakeUp.or(intakeDown).whileFalse(Commands.run(()->{
        m_intake.intakeMotor.setControl(new MotionMagicDutyCycle(m_intake.intakeMotor.getPosition().getValueAsDouble()));
        m_intake.feederWheel.set(0);},m_intake));

    intakeDown.whileTrue(Commands.run(()->{                
      m_intake.intakeMotor.setControl(new MotionMagicDutyCycle(Constants.INTAKE_DOWN_POSITION));
      m_intake.feederWheel.set((Math.abs(m_intake.intakeMotor.getPosition().getValueAsDouble()-Constants.INTAKE_DOWN_POSITION)<.7?Constants.MAX_FLYWHEEL_VOLTAGE:0));
      }, m_intake));
    shooterTrigger.whileTrue(shootCommand);
    swerveLockButton.whileTrue(Commands.run(() -> m_drivetrain.swerveDrive.lockPose()));
    swerveLockButton.onFalse(Commands.run(() -> driveCommand.execute(), m_drivetrain));
    
    alignIntakeTrigger.onTrue(Commands.runOnce(()->m_intake.intakeMotor.setPosition(-5.9)));
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

  PIDController velocityController = new PIDController(5.5, 0, 0);

  public void periodic(){
    joystickDegree = SmartDashboard.getNumber("Joystick Degree", joystickDegree);
    // System.out.println(m_drivetrain.swerveDrive.getPose());
    // System.out.println(m_drivetrain.orientationError());
    // System.out.println("Angle: " + m_drivetrain.hubAngle());
    // System.out.println("X: " + m_drivetrain.swerveDrive.getPose().getX());
    // System.out.println("Y: " + m_drivetrain.swerveDrive.getPose().getY());
    SmartDashboard.putNumber("nodsjfodjfadf", m_intake.intakeMotor.getPosition().getValueAsDouble());
    // System.out.println(m_drivetrain.getHeadingError());
    // SmartDashboard.putNumber("rotation", m_shooter.getOrientationError());
    // SmartDashboard.putNumber("tangential", m_shooter.getTangentialVelocity());
    // SmartDashboard.putNumber("radial", m_shooter.getRadialVelocity());
    SmartDashboard.putNumber("dial RPM",flywheelDial.getAsDouble());
    SmartDashboard.putNumber("actual RPM", m_shooter.shooterMotor.getVelocity().getValueAsDouble());
    if(isShooterManual){
      // System.out.println(flywheelDial.getAsDouble());//*Constants.MAX_FLYWHEEL_VOLTAGE
      m_shooter.shooterMotor.set(velocityController.calculate(m_shooter.shooterMotor.getVelocity().getValueAsDouble(), flywheelDial.getAsDouble()*200));
    } else {
      m_shooter.shooterMotor.set(0);
    }
    SmartDashboard.putNumber("Velocity", Math.sqrt(Math.pow(m_drivetrain.swerveDrive.getFieldVelocity().vxMetersPerSecond,2)+Math.pow(m_drivetrain.swerveDrive.getFieldVelocity().vyMetersPerSecond,2)));
    SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    SmartDashboard.putNumber("intake wrist", m_intake.intakeMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("X", m_drivetrain.swerveDrive.getPose().getX());
    SmartDashboard.putNumber("Y", m_drivetrain.swerveDrive.getPose().getY());
    SmartDashboard.putNumber("Z", m_drivetrain.swerveDrive.getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("Theta", m_drivetrain.swerveDrive.getYaw().getDegrees());
    SmartDashboard.putBoolean("ShootMode", robotState==RobotState.SHOOT);
    SmartDashboard.putBoolean("isTrenching", isTrenchLock);
  }

  public enum RobotState{
    NEUTRAL,
    INTAKE,
    SHOOT,
    OUTTAKE,
    DEFENSE
  }
}
