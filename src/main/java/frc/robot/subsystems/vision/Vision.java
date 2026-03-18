package frc.robot.subsystems.vision;

import java.util.*;
import java.util.function.BiConsumer;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public final class Vision extends SubsystemBase {
    public Camera shutter;
    public Camera ardu;    
    public boolean visionOn = true;
    public Drivetrain drivetrain;
    public boolean tagSeenInAuto;
    public boolean tagSeenInTeleop;
    public SwerveDrivePoseEstimator autoEstimator;
    public AHRS navx;
    BiConsumer<Pose2d, Double> updateDrivetrain;
    public Vision(BiConsumer<Pose2d, Double> updateDrivetrain, Drivetrain drivetrain){
        this.drivetrain = drivetrain;
        shutter = new Camera("Shutter623", new Transform3d(0.2632456, 0.2980182, 0.512445, new Rotation3d(0,Math.toRadians(-18.3),Math.toRadians(11))));
        ardu = new Camera("Arducam623", new Transform3d(0.2630932, -0.300228, 0.5041392, new Rotation3d(0,Math.toRadians(-9.2),Math.toRadians(-10))));
        this.updateDrivetrain = updateDrivetrain;
        navx = new AHRS(NavXComType.kMXP_SPI);
        autoEstimator = new SwerveDrivePoseEstimator(   
         new SwerveDriveKinematics(Constants.swerveDriveTranslations),
         navx.getRotation2d(),
         drivetrain.swerveDrive.getModulePositions(),
         new Pose2d()
        );
    }
    //This method updates Drivetrain's odometry to align with field-relative measurements. It is internally called periodically, so there is no need to use this method
    public void periodic(){
        SmartDashboard.putNumber("navx reading", navx.getAngle());
        SmartDashboard.putNumber("yagsl reading", drivetrain.swerveDrive.getYaw().getDegrees());
        SmartDashboard.putBoolean("Vision", visionOn);
        // if(!tagSeenInAuto && !tagSeenInTeleop){
        // visionOn=false;
        
        if(
            !visionOn /* has vision been disabled? */
            || Math.abs(drivetrain.swerveDrive.getFieldVelocity().omegaRadiansPerSecond) >= Constants.MAX_STABLE_ANGULAR_VELOCITY /* is the bot rotating too fast? */
            ) return;
        List<VisionReading> readings = new ArrayList<>();
        for(VisionReading reading: shutter.estimatePose())
            readings.add(reading);
            // updateDrivetrain.accept(reading.getPose2d(), reading.getTimestampSeconds());
            for(VisionReading reading: ardu.estimatePose())
                readings.add(reading);
            if(DriverStation.isAutonomous()){
                autoEstimator.updateWithTime(Timer.getFPGATimestamp(), navx.getRotation2d(), drivetrain.swerveDrive.getModulePositions());
                tagSeenInAuto = tagSeenInAuto || readings.size()>0;
                for (VisionReading reading:readings)
                    autoEstimator.addVisionMeasurement(reading.getPose2d(), reading.getTimestampSeconds());        
            } else {// if tele
                tagSeenInTeleop = tagSeenInTeleop || readings.size()>0;
                for (VisionReading reading:readings)
                    updateDrivetrain.accept(reading.getPose2d(), reading.getTimestampSeconds());
            }
    }
    // returns the argument,
    public boolean toggleVision(boolean visionOn){
        if(visionOn==false && this.visionOn==true){
            disableVision();
        } else if(visionOn==true && this.visionOn==false){
            enableVision();
        }

        this.visionOn = visionOn;
        return visionOn;
    }
    public void disableVision(){

    }
    public void enableVision(){

    }


}
