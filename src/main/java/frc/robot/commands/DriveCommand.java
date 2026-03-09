package frc.robot.commands;

import java.util.HashSet;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer.RobotState;
import frc.robot.subsystems.Drivetrain;

public class DriveCommand extends Command{
     
    private Drivetrain drivetrain;
    private DoubleSupplier xTranslationSupplier;
    private DoubleSupplier yTranslationSupplier;
    private DoubleSupplier thetaTranslationSupplier;
    private PIDController thetaController;
    private Supplier<RobotState> stateSupplier;
    /**
     * Constructs a DriveCommand command
     * 
     * @param swerveDrive the swerveDrive instance
     * @param xDoubleSupplier {@link java.util.function.DoubleSupplier DoubleSupplier} that supplies the double value between [-1,1] for joystick x translation
     * @param yDoubleSupplier {@link java.util.function.DoubleSupplier DoubleSupplier} that supplies the double value between [-1,1] for joystick y translation
     * @param thetaDoubbleSupplier {@link java.util.function.DoubleSupplier DoubleSupplier} that supplies the double value between [-1,1] for joystick theta translation
     */ // fix docs 
    public DriveCommand(Drivetrain drivetrain, 
                        Supplier<RobotState> stateSupplier,
                        DoubleSupplier xTranslationSupplier, 
                        DoubleSupplier yTranslationSupplier, 
                        DoubleSupplier thetaTranslationSupplier, 
                        double kP,  
                        double kI, 
                        double kD
){

        this.drivetrain = drivetrain;
        this.stateSupplier = stateSupplier;
        this.xTranslationSupplier = xTranslationSupplier;
        this.yTranslationSupplier = yTranslationSupplier;
        this.thetaTranslationSupplier = thetaTranslationSupplier;
        this.thetaController = new PIDController(kP, kI, kD);
        
    }
    @Override
    public void initialize() {

    }
    @Override
    public void execute(){
        drivetrain.swerveDrive.driveFieldOriented(new ChassisSpeeds(

            deadzone(xTranslationSupplier.getAsDouble(),0.05)
              * drivetrain.swerveDrive.getMaximumChassisVelocity(),

            deadzone(yTranslationSupplier.getAsDouble(),0.05)
              * drivetrain.swerveDrive.getMaximumChassisVelocity(),

            (stateSupplier.get()==RobotState.OUTTAKE?
            
              echo():deadzone(thetaTranslationSupplier.getAsDouble(),0.05) * drivetrain.swerveDrive.getMaximumChassisAngularVelocity()
            )),
          new Translation2d()
        );
    }
    @Override
    public void end(boolean interrupted){
        
    } 

    public boolean isFinished(){
        
        return false;
    }
    public double echo(){
      drivetrain.distance();
      double a = drivetrain.orientationError();
      SmartDashboard.putNumber("angle error: ", a);
      double x = thetaController.calculate(a,0);
      // thetaController.enableContinuousInput(-180, 180); use this or not?
      //minimum compensation for the PID loop?
      // System.out.println("Calculated output: "+ x);
      return x;
    }
    @Override
    public boolean runsWhenDisabled(){
        return false;
    }

    public HashSet<Subsystem> getRequirements(){
        HashSet<Subsystem> req = new HashSet<>();
        req.add(drivetrain);
        return req;
    }

    public static double deadzone(double num, double deadband){
        if (Math.abs(num) < deadband)
          return 0.0;
        return num;
      }
 }
