package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class Vision {
    public static PhotonCamera camera = new PhotonCamera("leftCamera623");

    // Switch if necessary - field layout
     public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

     // Robot-relative position of camera
     public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

     public static PhotonPoseEstimator photonEstimator;

    
    private Vision(){
        photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);
    }

    public static Pose2d get


}
