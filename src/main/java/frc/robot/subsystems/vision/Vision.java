package frc.robot.subsystems.vision;

import java.util.*;
import java.util.function.BiConsumer;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public final class Vision extends SubsystemBase {
    public Camera shutter;
    public Camera ardu;    
    public boolean visionOn = false;
    BiConsumer<Pose2d, Double> updateDrivetrain;
    public Vision(BiConsumer<Pose2d, Double> updateDrivetrain){
        shutter = new Camera("Shutter623", new Transform3d(0.2632456, 0.2980182, 0.512445, new Rotation3d(0,Math.toRadians(-18.3),Math.toRadians(11))));
        ardu = new Camera("Arducam623", new Transform3d(0.2630932, -0.300228, 0.5041392, new Rotation3d(0,Math.toRadians(-9.2),Math.toRadians(-10))));
        this.updateDrivetrain = updateDrivetrain;
    }
    //This method updates Drivetrain's odometry to align with field-relative measurements. It is internally called periodically, so there is no need to use this method
    public void periodic(){
        SmartDashboard.putBoolean("Vision", visionOn);
        
        if(!visionOn /* has vision been disabled? */ /* is the bot rotating too fast? */) return;
        List<VisionReading> readings = new ArrayList<>();
        //add readings to the "readings" list
        for(VisionReading reading: shutter.estimatePose())
            readings.add(reading);
        for(VisionReading reading: ardu.estimatePose())
            readings.add(reading);
        //send "readings" to drivetrain
        for (VisionReading reading : readings)
            updateDrivetrain.accept(reading.getPose2d(), reading.getTimestampSeconds());
    }
    // returns the argument,
    // public boolean toggleVision(boolean visionOn){
    //     if(visionOn==false && this.visionOn==true){
    //         disableVision();
    //     } else if(visionOn==true && this.visionOn==false){
    //         enableVision();
    //     }

    //     this.visionOn = visionOn;
    //     return visionOn;
    // }
    // public void disableVision(){

    // }
    // public void enableVision(){

    // }


}
