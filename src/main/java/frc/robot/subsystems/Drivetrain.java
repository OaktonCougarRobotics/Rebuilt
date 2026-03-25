// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

// import javax.lang.model.util.ElementScanner14;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.parser.SwerveParser;

public class Drivetrain extends SubsystemBase {

  public SwerveDrive swerveDrive;
  public SysIdRoutine routine;

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
          File directory
    ) {
    try{
    swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED, new Pose2d());
    } catch (Exception e) {
      e.printStackTrace();
      throw new RuntimeException("File failed to be loaded");
    }
    // swerveDrive.getModuleMap().get("frontleft").setAngle(0); // recomment when working
    configureAuto();
    Math.sqrt(Math.pow(swerveDrive.getRobotVelocity().vxMetersPerSecond,2)+Math.pow(swerveDrive.getRobotVelocity().vyMetersPerSecond,2));

      routine = SwerveDriveTest.setDriveSysIdRoutine(new SysIdRoutine.Config(), this, swerveDrive,12.00,false);
      
    }
      public Command getSysIdCommand(){
        return SwerveDriveTest.generateSysIdCommand(routine, 1, 1.0, 1);
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
   * Returns the distance from the hub
   * @param x x position
   * @param y y position
   * @return distance
   */
  public double distance(){
    Pose2d currentPose = swerveDrive.getPose();
    double x = currentPose.getX();
    SmartDashboard.putNumber("botx", x);
    double y = currentPose.getY();
    SmartDashboard.putNumber("boty", y);
    if(DriverStation.getAlliance().get()==Alliance.Blue){
      SmartDashboard.putNumber("distance", Math.sqrt(Math.pow((x-Constants.blueHub.getX()),2) + Math.pow((y-Constants.blueHub.getY()),2)));
        return Math.sqrt(Math.pow((x-Constants.blueHub.getX()),2) + Math.pow((y-Constants.blueHub.getY()),2));
    }
        else{
          SmartDashboard.putNumber("distance", Math.sqrt((x-Constants.redHub.getX())*(x-Constants.redHub.getX())+(y-Constants.redHub.getY())*(y-Constants.redHub.getY())));
      return Math.sqrt((x-Constants.redHub.getX())*(x-Constants.redHub.getX())+(y-Constants.redHub.getY())*(y-Constants.redHub.getY()));
        }
    }
    public double distanceToRPM(){
      double[] function = {9.08571,29.14286,-3.42857};//0th coeff, 1st, 2nd, etc
      double distance = distance();
      double sum = 0;
      for(int i = 0;i < function.length; i++){
        sum += function[i]*Math.pow(distance,i);
      }
      SmartDashboard.putNumber("regression Output", sum);
      return sum;
    }
   /**
     * Finds the error in orientation of the bot to the hub based on position and actual orientation
     * @param real the actual orientation of the bot IN DEGREES
     * @param x the relative x position from the hub
     * @param y the relative y position from the hub
     * @return real orientation minus expected orientation IN DEGREES
     */
    // public double orientationError() {
    //   Pose2d currentPose = swerveDrive.getPose();
    //   double x = currentPose.getX();
    //   double y = currentPose.getY();
    //   double currentAngle = currentPose.getRotation().getDegrees() % 360;//fr
    //   double angleToHub = Math.toDegrees(Math.atan((y-Constants.redHub.getY())/Math.abs(x-Constants.redHub.getX())) % 360);// add test case for right above/under when x=0

    //   double error = currentAngle - angleToHub;
    //   // error = -1 * ((error + 180) % 360);
    //   if (error > 0) {
    //     error -= 180;
    //   } else {
    //     error += 180;
    //   }
    //   return error;
    // }
    // TRY THIS METHOD
    public double getHeadingError() {
        Translation2d target = pickTarget();
        Pose2d currentPose = swerveDrive.getPose();

        double dx = target.getX() - currentPose.getX();
        double dy = target.getY() - currentPose.getY();

        double targetAngle = Math.toDegrees(Math.atan2(dy, dx));

        double error = targetAngle - currentPose.getRotation().getDegrees();;
        error = (error+360)%360; 
        // error = (error + 180) % 360;
        if (error > 180) error -= 360;
        // error -= 180;
        // System.out.println(error);
        return error;
    }
    public double hubAngle() {
      Pose2d currentPose = swerveDrive.getPose();
      double x = currentPose.getX();
      double y = currentPose.getY();
      //Translation2d target
      double angleToHub = Math.toDegrees(Math.atan((y-Constants.redHub.getY())/(x-Constants.redHub.getX())) % 360);
      return angleToHub;
    }

    public double tangentialVelocity() {
      ChassisSpeeds velocity = swerveDrive.getFieldVelocity();
      double vx = velocity.vxMetersPerSecond;
      double vy = velocity.vyMetersPerSecond;
      double velocityAngle = Math.atan(Math.toRadians(vy/vx));
      double speed = Math.sqrt(Math.pow(vx, 2) + Math.pow(vy,2));
      double vt = speed * Math.cos(Math.toRadians(90 - hubAngle() - (180 - velocityAngle)));
      return vt;
    }

    public double radialVelocity() {
      ChassisSpeeds velocity = swerveDrive.getFieldVelocity();
      double vx = velocity.vxMetersPerSecond;
      double vy = velocity.vyMetersPerSecond;
      double velocityAngle = Math.atan(Math.toRadians(vy/vx));
      double speed = Math.sqrt(Math.pow(vx, 2) + Math.pow(vy,2));
      double vt = speed * Math.sin(Math.toRadians(90 - hubAngle() - (180 - velocityAngle)));
      return vt;
    }
    public Translation2d pickTarget(){
      Pose2d pose = swerveDrive.getPose();
      if(DriverStation.getAlliance().get()==Alliance.Red){
        if(pose.getX()>11.99) // red alliance and in alliance zone
          return Constants.redHub;
        else if (pose.getY()>4.03) // red alliance and not in alliance zone
          return Constants.redPassUp;
        else 
          return Constants.redPassDown;
      }
      else {
        if(pose.getX()<4.55) // blue alliance and in alliance zone
          return Constants.blueHub; 
        else if (pose.getY()>4.03) // blue alliance and not in alliance zone
          return Constants.bluePassUp;
        else
          return Constants.bluePassDown;
      }
    }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("distance from hub", distance());
  }

  private Pose2d getPose(){
    return swerveDrive.getPose();
  }
  private void driveRobotRelative(ChassisSpeeds speeds){
    swerveDrive.drive(speeds, false, new Translation2d());
    swerveDrive.drive(speeds);
  }
  public void resetPose(Pose2d pose){
    swerveDrive.resetOdometry(pose);
  }
  private ChassisSpeeds getRobotRelativeSpeeds(){
    return swerveDrive.getRobotVelocity();
  }
  public void zeroGyro(){
    swerveDrive.zeroGyro();
  }
  public void resetEverything(){
    resetPose(new Pose2d());
        zeroGyro();
  }
}
