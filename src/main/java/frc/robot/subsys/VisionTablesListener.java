package frc.robot.subsys;

import java.util.ArrayList;
import java.util.LinkedList;

import edu.wpi.first.math.ComputerVisionUtil;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.CoordinateSystem;
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

    private Transform3d cam1Transform = new Transform3d(new Translation3d(0.3556, 0, 0.47), new Rotation3d(0, Math.toRadians(-15), 0));

    private static LinkedList<AprilTagDetection> tagDetections = new LinkedList<AprilTagDetection>();

    private boolean tagVisible;

    public VisionTablesListener() {
        networkTable = NetworkTableInstance.getDefault();
        visionTable = networkTable.getTable("Vision");

        // tagIDSub1 = visionTable.getIntegerArrayTopic("IDs").subscribe(new long[] {});
        // x1Sub = visionTable.getDoubleArrayTopic("X Coords").subscribe(new double[] {});
        // y1Sub = visionTable.getDoubleArrayTopic("Y Coords").subscribe(new double[] {});
        // z1Sub = visionTable.getDoubleArrayTopic("Z Coords").subscribe(new double[] {});
        // yaw1Sub = visionTable.getDoubleArrayTopic("Yaws").subscribe(new double[] {});
        // pitch1Sub = visionTable.getDoubleArrayTopic("Pitches").subscribe(new double[] {});
        // roll1Sub = visionTable.getDoubleArrayTopic("Rolls").subscribe(new double[] {});
        // timestamp1Sub = visionTable.getDoubleArrayTopic("Timestamps").subscribe(new double[] {});
        tag1Sub = visionTable.getStringArrayTopic("Serialized Tags").subscribe(new String[] {});
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
            Pose3d tagFieldPos = getBestTagAbsPos(tagDetections.get(i).getID());
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


    public Pose3d getBestTagAbsPos(int id) {

        // format: [x, y, z, rot]
        // ID X Y Z Rotation
        // 1 593.68 9.68 53.38 120°
        // 2 637.21 34.79 53.38 120°
        // 3 652.73 196.17 57.13 180°
        // 4 652.73 218.42 57.13 180°
        // 5 578.77 323.00 53.38 270°
        // 6 72.5 323.00 53.38 270°
        // 7 -1.50 218.42 57.13 0°
        // 8 -1.50 196.17 57.13 0°
        // 9 14.02 34.79 53.38 60°
        // 10 57.54 9.68 53.38 60°
        // 11 468.69 146.19 52.00 300°
        // 12 468.69 177.10 52.00 60°
        // 13 441.74 161.62 52.00 180°
        // 14 209.48 161.62 52.00 0°
        // 15 182.73 177.10 52.00 120°
        // 16 182.73 146.19 52.00 240°

        Pose3d bestTagAbsPos;
        switch (id) {
            case 1:
                bestTagAbsPos = new Pose3d(new Translation3d(15.079472, 0.245872, 1.355852), new Rotation3d(0, 0, Math.toRadians(120)));
                break;
            case 2:
                bestTagAbsPos = new Pose3d(new Translation3d(16.185134, 0.883666, 1.355852), new Rotation3d(0, 0, Math.toRadians(120)));
                break;
            case 3:
                bestTagAbsPos = new Pose3d(new Translation3d(16.579342, 4.982718, 1.451102), new Rotation3d(0, 0, Math.toRadians(180)));
                break;
            case 4:
                bestTagAbsPos = new Pose3d(new Translation3d(16.579342, 5.547868, 1.451102), new Rotation3d(0, 0, Math.toRadians(180)));
                break;
            case 5:
                bestTagAbsPos = new Pose3d(new Translation3d(14.700758, 8.2042, 1.355852), new Rotation3d(0, 0, Math.toRadians(270)));
                break;
            case 6:
                bestTagAbsPos = new Pose3d(new Translation3d(1.8415, 8.2042, 1.355852), new Rotation3d(0, 0, Math.toRadians(-90)));
                break;
            case 7:
                bestTagAbsPos = new Pose3d(new Translation3d(-0.0381, 5.547868, 1.451102), new Rotation3d(0, 0, Math.toRadians(0)));
                break;
            case 8:
                bestTagAbsPos = new Pose3d(new Translation3d(-0.0381, 4.982718, 1.451102), new Rotation3d(0, 0, Math.toRadians(0)));
                break;
            case 9:
                bestTagAbsPos = new Pose3d(new Translation3d(0.356108, 0.883666, 1.355852), new Rotation3d(0, 0, Math.toRadians(60)));
                break;
            case 10:
                bestTagAbsPos = new Pose3d(new Translation3d(1.461516, 0.245872, 1.355852), new Rotation3d(0, 0, Math.toRadians(60)));
                break;
            case 11:
                bestTagAbsPos = new Pose3d(new Translation3d(11.904726, 3.713226, 1.3208), new Rotation3d(0, 0, Math.toRadians(300)));
                break;
            case 12:
                bestTagAbsPos = new Pose3d(new Translation3d(11.904726, 4.49834, 1.3208), new Rotation3d(0, 0, Math.toRadians(60)));
                break;
            case 13:
                bestTagAbsPos = new Pose3d(new Translation3d(11.220196, 4.105148, 1.3208), new Rotation3d(0, 0, Math.toRadians(180)));
                break;
            case 14:
                bestTagAbsPos = new Pose3d(new Translation3d(5.320792, 4.105148, 1.3208), new Rotation3d(0, 0, Math.toRadians(0)));
                break;
            case 15:
                bestTagAbsPos = new Pose3d(new Translation3d(4.641342, 4.49834, 1.3208), new Rotation3d(0, 0, Math.toRadians(120)));
                break;
            case 16:
                bestTagAbsPos = new Pose3d(new Translation3d(4.641342, 3.713226, 1.3208), new Rotation3d(0, 0, Math.toRadians(240)));
                break;
            default:
                bestTagAbsPos = null;
        }

        return bestTagAbsPos;
    }
}
