package frc.lib.pathplanner;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Collections;
import java.util.List;
import java.util.stream.Stream;

import edu.wpi.first.wpilibj.Filesystem;

public class PathPlannerUtil {
    public static List<String> getExistingPaths() {
        var path = Path.of(Filesystem.getDeployDirectory().getAbsolutePath(), "pathplanner");
        try (Stream<Path> stream = Files.walk(path)) {
            return stream
                    .filter(x -> getFileExtension(x.toFile()).equals(".path"))
                    .map(x -> getFileStem(x.toFile()))
                    .toList();
        } catch (IOException e) {
            return Collections.emptyList();
        }
    }

    private static String getFileStem(File file) {
        try {
            String name = file.getName();
            return name.substring(0, name.lastIndexOf("."));
        } catch (Exception e) {
            return "";
        }
    }

    private static String getFileExtension(File file) {
        try {
            String name = file.getName();
            return name.substring(name.lastIndexOf("."));
        } catch (Exception e) {
            return "";
        }
    }
}
