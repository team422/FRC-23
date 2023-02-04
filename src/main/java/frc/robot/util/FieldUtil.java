package frc.robot.util;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.FullSwerveBase;

public class FieldUtil {
    public static final String kDefaultFieldName = "Field";
    private static Map<String, FieldUtil> fieldInstances = new HashMap<>();

    private final Field2d field = new Field2d();

    private FieldUtil(String name) {
        SmartDashboard.putData(name, field);
    }

    public static synchronized FieldUtil getDefaultField() {
        return getField(kDefaultFieldName);
    }

    public static synchronized FieldUtil getField(String name) {
        if (!fieldInstances.containsKey(name)) {
            fieldInstances.put(name, new FieldUtil(name));
        }

        return fieldInstances.get(name);
    }

    public void setObjects(Map<String, Pose2d> objects) {
        for (var entry : objects.entrySet()) {
            String objectName = entry.getKey();
            Pose2d objectPose = entry.getValue();
            field.getObject(objectName).setPose(objectPose);
        }
    }

    public void setObjects3d(Map<String, Pose3d> objects) {
        for (var entry : objects.entrySet()) {
            String objectName = entry.getKey();
            Pose2d objectPose = entry.getValue().toPose2d();
            field.getObject(objectName).setPose(objectPose);
        }
    }

    public void setObjectPose(String name, Pose2d pose) {
        field.getObject(name).setPose(pose);
    }

    public void setObjectPoses(String name, Pose2d... poses) {
        field.getObject(name).setPoses(poses);
    }

    public void setTrajectory(String name, Trajectory trajectory) {
        field.getObject(name).setTrajectory(trajectory);
    }

    public void updateRobotPose(Pose2d pose) {
        field.setRobotPose(pose);
    }

    public void setSwerveRobotPose(Pose2d pose, FullSwerveBase drive) {
        updateRobotPose(pose);

        var states = drive.getSwerveStates();
        var translations = DriveConstants.kModuleTranslations;

        Pose2d[] modulePoses = new Pose2d[states.length];

        for (int i = 0; i < states.length; i++) {
            var translation = translations[i];
            var rotation = states[i].angle;

            modulePoses[i] = pose.transformBy(new Transform2d(translation, rotation));
        }

        setObjectPoses("RobotSwerveModules", modulePoses);
    }

    public Pose2d getObjectPose(String name) {
        return field.getObject(name).getPose();
    }

    public void removeObject(String name) {
        field.getObject(name).setPoses();
    }
}
