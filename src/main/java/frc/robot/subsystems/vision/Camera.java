package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Camera extends SubsystemBase{
    private final String name;
    private final PhotonCamera cam;
    private final PhotonPoseEstimator poseEstimator;

    public Camera(String name, Transform3d kRobotToCam){
        this.name = name;
        this.cam = new PhotonCamera(name);
        this.poseEstimator = new PhotonPoseEstimator(Constants.kTagLayout, kRobotToCam);
    }
    /** 
     *  
     * @return a {@link List<VisionReading>} of all the different readings taken by the camera at a specific instance in time
     */
    List<VisionReading> estimatePose() {
        List<PhotonPipelineResult> results = cam.getAllUnreadResults();
        List<VisionReading> readings = new ArrayList<>();
        for(PhotonPipelineResult res: results){
            if(res.getMultiTagResult().isPresent()){
                Optional<EstimatedRobotPose> x = poseEstimator.estimateCoprocMultiTagPose(res);
            if(x.isPresent()) 
                readings.add(new VisionReading(x.get().estimatedPose, res.getTimestampSeconds()));
            }
        }
        return readings;
    }
    @Override
    /** 
     * Periodically calls the estimatePose() method, ensuring constant updating of odometry.
     */
    public void periodic(){
    estimatePose();
    }
    /** 
     * @return boolean value representing whether the camera is connected (i.e. the camera is sending data).
     */
    public boolean isConnected(){
        return cam.isConnected();
    }
    public String toString(){
        return "{Name:"+name+", isActive: "+isConnected()+"}";
    }
}
class VisionReading{
    private Pose3d pose;
    private double timestamp;
    public VisionReading(Pose3d pose, double timestamp){
        this.pose = pose;
        this.timestamp = timestamp;
    }
    public Pose2d getPose2d(){
        return pose.toPose2d();
    }
    public double getTimestampSeconds(){
        return timestamp;
    }
}

