package frc.robot.subsystems.vision;

import java.util.function.BiConsumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class Vision extends SubsystemBase {
    public Camera shutter;
    public Camera ardu;    
    BiConsumer<Pose2d, Double> updateDrivetrain;
    public Vision(BiConsumer<Pose2d, Double> updateDrivetrain){
        shutter = new Camera("Shutter623", new Transform3d(0.314325, 0.066675, 0.200025, new Rotation3d(0,Math.toRadians(33.4),Math.toRadians(-13.5))));
        ardu = new Camera("Arducam623", new Transform3d(0.301625, 0.1397, 0.1524, new Rotation3d(0,Math.toRadians(24.7),Math.toRadians(16.7))));
        this.updateDrivetrain = updateDrivetrain;
    }
    public static VisionReading estimatePose(){
        return null;
    }
    public void periodic(){
        for(VisionReading reading: shutter.estimatePose())
            updateDrivetrain.accept(reading.getPose2d(), reading.getTimestampSeconds());
        for(VisionReading reading: ardu.estimatePose())
            updateDrivetrain.accept(reading.getPose2d(), reading.getTimestampSeconds());
    }


}
