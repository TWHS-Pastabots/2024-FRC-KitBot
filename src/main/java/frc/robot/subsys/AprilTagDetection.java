package frc.robot.subsys;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class AprilTagDetection {
    private int ID;
    private Transform3d transform;
    private double timestamp;
    private static Rotation3d EDN_TO_NWU = new Rotation3d(MatBuilder.fill(Nat.N3(), Nat.N3(), 0, 0, 1, -1, 0, 0, 0, -1, 0));

    public AprilTagDetection(String serial){
        String[] splitSerial = serial.split(" ");

        ID = Integer.parseInt(splitSerial[0]);

        Translation3d ednTrl = new Translation3d(
            Double.parseDouble(splitSerial[1]),
            Double.parseDouble(splitSerial[2]),
            Double.parseDouble(splitSerial[3])
        );
        Rotation3d ednRot = new Rotation3d(
            new Quaternion(
                Double.parseDouble(splitSerial[4]),
                Double.parseDouble(splitSerial[5]),
                Double.parseDouble(splitSerial[6]),
                Double.parseDouble(splitSerial[7])
            )
        );
        
        //Transfrom must be converted from EDN to NWU
        Rotation3d nwuRot = EDN_TO_NWU.unaryMinus().plus(ednRot.plus(EDN_TO_NWU));
        transform = new Transform3d(
            ednTrl.rotateBy(EDN_TO_NWU), 
            new Rotation3d(nwuRot.getX(), nwuRot.getY(), -nwuRot.getZ())
        );

        timestamp = Double.parseDouble(splitSerial[8]);
    }

    public int getID() {
        return ID;
    }
    
    public Transform3d getTransform() {
        return transform;
    }

    public double getTimeStamp() {
        return timestamp;
    }
}
