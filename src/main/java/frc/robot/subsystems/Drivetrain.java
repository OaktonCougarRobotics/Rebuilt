// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.HashSet;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class Drivetrain extends SubsystemBase {

  private SwerveDrive swerveDrive;

  private DoubleSupplier xTranslationSupplier, yTranslationSupplier, thetaTranslationSupplier;

   /**
   * Drivetrain Subsystem. Controls movement of robot
   *
   * @param directory {@link File} representing the 'swerve' folder with YAGSL .json files
   * @param xTranslationalSupplier {@link DoubleSupplier} with range [-1,1] representing magnitude of joystick movement in x direction
   * @param yTranslationalSupplier {@link DoubleSupplier} with range [-1,1] representing magnitude of joystick movement in y direction
   * @param thetaTranslationalSupplier {@link DoubleSupplier} with range [-1,1] representing magnitude of joystick angular movement
   * @return nothing
   */
  public Drivetrain(
          File directory,
          DoubleSupplier xTranslationSupplier, 
          DoubleSupplier yTranslationSupplier, 
          DoubleSupplier thetaTranslationSupplier
    ) {
      this.xTranslationSupplier = xTranslationSupplier;
      this.yTranslationSupplier = yTranslationSupplier;
      this.thetaTranslationSupplier = thetaTranslationSupplier;
    try{
    swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED, new Pose2d());
    } catch (Exception e) {
      e.printStackTrace();
      throw new RuntimeException("File failed to be loaded");
    }
    configureAuto();
      }
        
      private void configureAuto() {
        RobotConfig config;
        try{
          config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
         // Handle exception as needed
           throw new RuntimeException("Failed to grab Pathplanner from GUI");
        }

        AutoBuilder.configure(
          this::getPose,
          this::resetPose, 
          this::getRobotRelativeSpeeds, 
          this::driveRobotRelative,
          new PPHolonomicDriveController(
              new PIDConstants(5.0, 0.0, 0.0),
              new PIDConstants(5.0, 0.0, 0.0)
          ),
          config,
          () -> {
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            }, 
          this);
      }
    
      /**
   * Default Drive Command factory method.
   *
   * @return driveCommand
   */
  public Command driveCommand() {
    Command d = run(
        () -> {
          swerveDrive.driveFieldOriented(new ChassisSpeeds(
            deadzone(xTranslationSupplier.getAsDouble(),0.05) // x
              * swerveDrive.getMaximumChassisVelocity(),
            deadzone(yTranslationSupplier.getAsDouble(),0.05) // y
              * swerveDrive.getMaximumChassisVelocity(),
            deadzone(thetaTranslationSupplier.getAsDouble(),0.05) // theta
              * swerveDrive.getMaximumChassisAngularVelocity()),
          new Translation2d());
        });
  d.addRequirements(this);
  // HashSet<Subsystem> reqs = new HashSet<>();
  // reqs.add(this);
  // d.addRequirements(reqs);
  return d;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Velocity", swerveDrive.getRobotVelocity().toString());
    SmartDashboard.putNumber("xSpeed", xTranslationSupplier.getAsDouble()*-1);
    SmartDashboard.putNumber("ypeed", yTranslationSupplier.getAsDouble()*-1);
    SmartDashboard.putNumber("thetaSpeed", thetaTranslationSupplier.getAsDouble()*-1);
    System.out.println(((TalonFX)(swerveDrive.getModuleMap().get("frontleft").getDriveMotor().getMotor())).isAlive());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


  private Pose2d getPose(){
    return swerveDrive.getPose();
  }
  private void driveRobotRelative(ChassisSpeeds speeds){
    swerveDrive.drive(speeds, false, new Translation2d());
  }
  private void resetPose(Pose2d pose){
    swerveDrive.resetOdometry(pose);
  }
  private ChassisSpeeds getRobotRelativeSpeeds(){
    return swerveDrive.getRobotVelocity();
  }
  public void zeroGyro(){
    swerveDrive.zeroGyro();
  }
  private double deadzone(double num, double deadband){
    if (Math.abs(num) < deadband)
      return 0.0;
    return num;
  }
}
