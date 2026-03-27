// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private boolean wasLastAuto;
  private final RobotContainer m_robotContainer;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    m_robotContainer = new RobotContainer();

    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    // CameraServer.startAutomaticCapture();

    // addPeriodic(()->{
    //   // if(m_robotContainer.robotState.equals(RobotState.OUTTAKE)){
    //   //   m_robotContainer.driveCommand.execute();
    //   }
    // }
    // , .005, .005);
    // //Advantagekit initialization no line of code comes before this
    // Logger.recordMetadata("ProjectName", "Rebuilt"); // Set a metadata value
    // if (isReal()) {
    //     Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    //     Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    // } else {
    //     setUseTiming(false); // Run as fast as possible
    //     String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
    //     Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
    //     Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    // }
    // Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
    
    
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.m_shooter.indexMotor.set(0);
    m_robotContainer.m_shooter.shooterMotor.set(0);
    m_robotContainer.m_intake.feederWheel.set(0);
    // if(wasLastAuto && m_robotContainer.m_vision.visionOn){

    //   m_robotContainer.m_drivetrain.resetPose(m_robotContainer.m_vision.autoEstimator.getEstimatedPosition());
    // }
    
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.periodic();
    // m_robotContainer.jank.periodic();
    for ( String x: m_robotContainer.m_drivetrain.swerveDrive.getModuleMap().keySet()){
      SmartDashboard.putNumber(x+"offset", m_robotContainer.m_drivetrain.swerveDrive.getModuleMap().get(x).getAbsolutePosition());
    }

      SmartDashboard.putNumber("hgfhgf", m_robotContainer.m_drivetrain.swerveDrive.getModuleMap().get("frontleft").getAbsolutePosition());
  // SmartDashboard.putNumber("baked", m_robotContainer.m_drivetrain.swerveDrive.getModuleMap().get("frontright").getAbsolutePosition());

  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.resetPose();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    SmartDashboard.putString("Potato", m_robotContainer.m_drivetrain.swerveDrive.getPose().toString());

    // System.out.println(m_robotContainer.m_drivetrain.swerveDrive.getPose());

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
    m_robotContainer.periodic();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    wasLastAuto = true;
    SmartDashboard.putString("Potato", m_robotContainer.m_drivetrain.swerveDrive.getPose().toString());

    // System.out.println(m_robotContainer.m_drivetrain.swerveDrive.getPose());
    // System.out.println(m_robotContainer.m_drivetrain.swerveDrive.getPose());
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_robotContainer.periodic();
    wasLastAuto = false;
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
