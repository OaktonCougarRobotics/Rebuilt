// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.vision.Vision;

import java.io.File;


import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
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
  private final Drivetrain m_drivetrain;
  private final Vision m_vision;
  private final Joystick m_joystick = new Joystick(1);
  private final Command driveCommand;
  private final Trigger navxResetButton = new Trigger(() -> m_joystick.getRawButton(3));
  private final Trigger alignTrigger = new Trigger(() -> m_joystick.getRawButton(6));
  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    m_drivetrain = new Drivetrain(
      new File(Filesystem.getDeployDirectory(), "swerve"));
      m_vision = new Vision(m_drivetrain.swerveDrive::addVisionMeasurement);
    driveCommand = new DriveCommand(
      m_drivetrain,
      () -> robotState,
      () -> m_joystick.getRawAxis(1) * -1,
      () -> m_joystick.getRawAxis(0) * -1,
      () -> m_joystick.getRawAxis(2) * -1,
      null,//replace to getVisionWorking
    1.0,
      0,
      0);
    // Configure the trigger bindings
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
    
//m_drivetrain.driveCommand()
    navxResetButton.onTrue(Commands.runOnce(m_drivetrain::zeroGyro));
    alignTrigger.whileTrue(Commands.runOnce(() -> robotState = RobotState.OUTTAKE));
    alignTrigger.onFalse(Commands.runOnce(() -> robotState = RobotState.NEUTRAL));
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous

    return new PathPlannerAuto("sigma");
  }
  public void disabledPeriodic(){
    // Map<String, SwerveModule> x = m_drivetrain.swerveDrive.getModuleMap();

    // SmartDashboard.putNumber("Frontleft angle", x.get("frontleft").getRawAbsolutePosition());
    // SmartDashboard.putNumber("FrontRight angle", x.get("frontright").getRawAbsolutePosition());
    // SmartDashboard.putNumber("Backleft angle", x.get("backleft").getRawAbsolutePosition());
    // SmartDashboard.putNumber("BackRight angle", x.get("backright").getRawAbsolutePosition());
  }
  public void periodic(){
    System.out.println(m_drivetrain.swerveDrive.getPose());
    // Map<String, SwerveModule> x = m_drivetrain.swerveDrive.getModuleMap();
    // SmartDashboard.putNumber("Frontleft angle", x.get("frontleft").getRawAbsolutePosition());
    // SmartDashboard.putData("OFFSET TESTING", new Sendable(){
    //   public void initSendable(SendableBuilder builder){
    //     builder.addDoubleProperty("frontrightabsolute", () ->x.get("frontright").getRawAbsolutePosition(),null);
    //   }
    // });
    // System.out.println(x.get("frontright").getRawAbsolutePosition());
    // System.out.println(x.get("frontright").getAbsoluteEncoder().getAbsolutePosition());
    // SmartDashboard.putNumber("BackRight angle", x.get("backright").getRawAbsolutePosition());

  }
  public enum RobotState{
    NEUTRAL,
    INTAKE,
    OUTTAKE
  }
}

