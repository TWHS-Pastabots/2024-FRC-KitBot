package frc.robot.subsys;

import java.util.ArrayList;
import java.util.LinkedList;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArraySubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionTablesListener {
    private static VisionTablesListener instance;

    private NetworkTableInstance networkTable;
    private NetworkTable visionTable;
    private StringArraySubscriber tag1Sub;

    private Transform3d cam1toRobot = new Transform3d(new Translation3d(-0.3556, 0, -0.47), new Rotation3d(0, Math.toRadians(165), 0));

    private static LinkedList<AprilTagDetection> tagDetections = new LinkedList<AprilTagDetection>();
    private boolean tagVisible;
    private AprilTagFieldLayout tagLayout;

    public VisionTablesListener() {
        networkTable = NetworkTableInstance.getDefault();
        visionTable = networkTable.getTable("Vision");
        tag1Sub = visionTable.getStringArrayTopic("Serialized Tags").subscribe(new String[] {});

        try {
            tagLayout = AprilTagFieldLayout.loadFromResource("2024-crescendo.json");
        } catch (IOException e){

        }
    }

    public void putInfoOnDashboard() {
        if(tag1Sub.get().length > 0) {
            tagVisible = true;
        }
        else {
            tagVisible = false;
        }

        SmartDashboard.putBoolean("Tag in Sight", tagVisible);
    }

    public static VisionTablesListener getInstance() {
        if (instance == null)
            instance = new VisionTablesListener();
        return instance;
    }

    public Pose2d[] getCam1Poses() {
        tagDetections.clear();
        String[] serializedTags = tag1Sub.get();
        for(String tagSerial: serializedTags) {
            tagDetections.add(new AprilTagDetection(tagSerial));
        }
        
        if(tagDetections == null || tagDetections.size() == 0)
            return null;

        Pose2d[] poses = new Pose2d[tagDetections.size()];
        for(int i = 0; i < poses.length; i++) {
            Pose3d tagFieldPos = tagLayout.getTagPose(tagDetections.get(i).getID()).get();
            Transform3d robotPos = tagDetections.get(i).getTransform().plus(cam1Transform);
            robotPos = new Transform3d(
                new Translation3d(
                    robotPos.getX(),
                    -robotPos.getY(),
                    -robotPos.getZ()
                ),
                robotPos.getRotation()
            );
            poses[i] = tagFieldPos.transformBy(robotPos).toPose2d();
        }
        return poses;
    }
}
