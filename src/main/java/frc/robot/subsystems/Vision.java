package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class Vision {
    public static PhotonCamera camera = new PhotonCamera("leftCamera623");

    // Switch if necessary - field layout
     public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

     // Robot-relative position of camera
     public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

     public static PhotonPoseEstimator poseEstimator= new PhotonPoseEstimator(kTagLayout, kRobotToCam);

    
    private Vision(){}
    //janky method only works properly when there is one apriltag
<<<<<<< HEAD
    public static Pose2d getEstimatedPosition(){
        return null;
    }
    public void periodic(){
        // poseEstimator.estimateCoprocMultiTagPose(camera.)
    }

=======
    public static Optional<Pose2d> getEstimatedPosition(){

        List<PhotonPipelineResult> results= camera.getAllUnreadResults();
        for(var result : results){
            var multiTagResult = result.getMultiTagResult();
            if(multiTagResult.isPresent()){
                Translation3d trans =  multiTagResult.get().estimatedPose.best.getTranslation();
                Rotation3d rot =  multiTagResult.get().estimatedPose.best.getRotation();
                return Optional.of(new Pose2d(trans.getX(), trans.getY(), new Rotation2d(rot.getMeasureAngle())));
            }
        }
        return Optional.empty();
        // throw new RuntimeException("no results found!");
    }
>>>>>>> 94d356af26bf9f041b2a20edffb3e7187662c4d3
}
