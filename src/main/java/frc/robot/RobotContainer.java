// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.vision.Vision;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
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
  private final Joystick m_joystick = new Joystick(1);
  final Command driveCommand;
  private final Trigger navxResetButton = new Trigger(() -> m_joystick.getRawButton(3));
  private final Trigger sysIdButton = new Trigger(() -> m_joystick.getRawButton(4));
  Command sysRoutine;

  private final Trigger alignTrigger = new Trigger(() -> m_joystick.getRawButton(6));

  private SendableChooser<Command> autoChooser;
  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    m_drivetrain = new Drivetrain(
      new File(Filesystem.getDeployDirectory(), "swerve"));
      m_vision = new Vision(m_drivetrain.swerveDrive::addVisionMeasurement, m_drivetrain);
    driveCommand = new DriveCommand(
      m_drivetrain,
      () -> robotState,
      () -> m_joystick.getRawAxis(1) * (DriverStation.getAlliance().get()==Alliance.Blue?-1:1),
      () -> m_joystick.getRawAxis(0) * (DriverStation.getAlliance().get()==Alliance.Blue?-1:1),
      () -> m_joystick.getRawAxis(2) * -1,
      null,//replace to getVisionWorking
    0.2,
      0.0,
      0.0);
      // NamedCommands.registerCommand("Potato", Commands.print("HKFJSDHFKJDSHFKJSDHFKSJDFHKSDJFHSDKJFHSDKJFHSDKJFHSDFKJSDHFKJSDHFKJSDFHSKJFHSKJDFHSKDJFHSDKJFHSDKFJSDHFKSJDFKSJDFHSDKJFHSDKJFSDH"));
    // Configure the trigger bindings
    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier((stream) -> true? //fix this
      stream.filter(auto -> auto.getName().startsWith("")):stream);
    SmartDashboard.putData("Auto Chooser", autoChooser);
    sysRoutine = m_drivetrain.getSysIdCommand();
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
    navxResetButton.onTrue(Commands.runOnce(m_drivetrain::zeroGyro));
    alignTrigger.whileTrue(Commands.runOnce(() -> robotState = RobotState.OUTTAKE));
    alignTrigger.whileFalse(Commands.runOnce(() -> robotState = RobotState.NEUTRAL));
    sysIdButton.onTrue(sysRoutine);
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
    return new PathPlannerAuto("Joshua's wrath");
  }
  public void disabledPeriodic(){
    //Implement as required
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
    

  }
  public enum RobotState{
    NEUTRAL,
    INTAKE,
    OUTTAKE
  }
}

