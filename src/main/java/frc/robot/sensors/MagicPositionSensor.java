package frc.robot.sensors;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import frc.robot.util.Field2d;

public class MagicPositionSensor {
    private final Field2d field2d;
    
    public MagicPositionSensor(Field2d field2d) {
        this.field2d = field2d;
    }

    public Pose2d getRobotPosition() {
        return field2d.getRobotPose();
    }

}