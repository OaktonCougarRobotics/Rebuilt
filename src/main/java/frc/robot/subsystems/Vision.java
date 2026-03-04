package frc.robot.subsystems;

import java.io.IOException;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.proto.Photon;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Vision {
    public static PhotonPoseEstimator poseEstimator;
    private Vision(){}

    public static void init(String pathName, Transform3d robotToCameraDistance) throws IOException {
        poseEstimator = new PhotonPoseEstimator(
        new AprilTagFieldLayout(pathName), 
        null);
    }
    
    public static Angle getDesiredAngle(Pose2d robotPose, Pose2d targetPose) {
        /* Process error between robot angle and certain setpoint on field */
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
            return null;// add 180 to calculation
        else /*calculate normal error */
            return null;
    }
}
