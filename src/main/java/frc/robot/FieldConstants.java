// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise.
 *
 * <p>All translations and poses are stored with the origin at the rightmost point on the BLUE
 * ALLIANCE wall. Use the {@link #allianceFlip(Translation2d)} and {@link #allianceFlip(Pose2d)}
 * methods to flip these values based on the current alliance color.
 */
public final class FieldConstants {
  public static final boolean isWPIField = false; // Red alliance

  public static final double fieldLength = Units.inchesToMeters(651.25);
  public static final double fieldWidth = Units.inchesToMeters(315.5) + (isWPIField ? Units.inchesToMeters(3.0) : 0.0);
  public static final double tapeWidth = Units.inchesToMeters(2.0);

  // Dimensions for community and charging station, including the tape.
  public static final class Community {
    // Region dimensions
    public static final double innerX = 0.0;
    public static final double midX = Units.inchesToMeters(132.375); // Tape to the left of charging station
    public static final double outerX = Units.inchesToMeters(193.25); // Tape to the right of charging station
    public static final double leftY = Units.feetToMeters(18.0);
    public static final double midY = leftY - Units.inchesToMeters(59.39) + tapeWidth;
    public static final double rightY = 0.0;
    public static final Translation2d[] regionCorners = new Translation2d[] {
        new Translation2d(innerX, rightY),
        new Translation2d(innerX, leftY),
        new Translation2d(midX, leftY),
        new Translation2d(midX, midY),
        new Translation2d(outerX, midY),
        new Translation2d(outerX, rightY),
    };

    // Charging station dimensions
    public static final double chargingStationLength = Units.inchesToMeters(76.125);
    public static final double chargingStationWidth = Units.inchesToMeters(97.25);
    public static final double chargingStationOuterX = outerX - tapeWidth;
    public static final double chargingStationInnerX = chargingStationOuterX - chargingStationLength;
    public static final double chargingStationLeftY = midY - tapeWidth;
    public static final double chargingStationRightY = chargingStationLeftY - chargingStationWidth;
    public static final Translation2d[] chargingStationCorners = new Translation2d[] {
        new Translation2d(chargingStationInnerX, chargingStationRightY),
        new Translation2d(chargingStationInnerX, chargingStationLeftY),
        new Translation2d(chargingStationOuterX, chargingStationRightY),
        new Translation2d(chargingStationOuterX, chargingStationLeftY)
    };

    // Cable bump
    public static final double cableBumpInnerX = innerX + Grids.outerX + Units.inchesToMeters(95.25);
    public static final double cableBumpOuterX = cableBumpInnerX + Units.inchesToMeters(7);
    public static final Translation2d[] cableBumpCorners = new Translation2d[] {
        new Translation2d(cableBumpInnerX, 0.0),
        new Translation2d(cableBumpInnerX, chargingStationRightY),
        new Translation2d(cableBumpOuterX, 0.0),
        new Translation2d(cableBumpOuterX, chargingStationRightY)
    };
  }

  // Dimensions for grids and nodes
  public static final class Grids {
    // X layout
    public static final double outerX = Units.inchesToMeters(54.25);
    public static final double lowX = outerX - (Units.inchesToMeters(14.25) / 2.0); // Centered when under cube nodes
    public static final double midX = outerX - Units.inchesToMeters(22.75);
    public static final double highX = outerX - Units.inchesToMeters(39.75);

    // Y layout
    public static final int nodeRowCount = 9;
    public static final double[] nodeY = isWPIField
        ? new double[] {
            Units.inchesToMeters(20.19 + 22.0 * 0),
            Units.inchesToMeters(20.19 + 22.0 * 1),
            Units.inchesToMeters(20.19 + 22.0 * 2),
            Units.inchesToMeters(20.19 + 22.0 * 3),
            Units.inchesToMeters(20.19 + 22.0 * 4),
            Units.inchesToMeters(20.19 + 22.0 * 5),
            Units.inchesToMeters(20.19 + 22.0 * 6),
            Units.inchesToMeters(20.19 + 22.0 * 7),
            Units.inchesToMeters(20.19 + 22.0 * 8 + 2.5)
        }
        : new double[] {
            Units.inchesToMeters(20.19 + 22.0 * 0),
            Units.inchesToMeters(20.19 + 22.0 * 1),
            Units.inchesToMeters(20.19 + 22.0 * 2),
            Units.inchesToMeters(20.19 + 22.0 * 3),
            Units.inchesToMeters(20.19 + 22.0 * 4),
            Units.inchesToMeters(20.19 + 22.0 * 5),
            Units.inchesToMeters(20.19 + 22.0 * 6),
            Units.inchesToMeters(20.19 + 22.0 * 7),
            Units.inchesToMeters(20.19 + 22.0 * 8)
        };

    // Z layout
    public static final double cubeEdgeHigh = Units.inchesToMeters(3.0);
    public static final double highCubeZ = Units.inchesToMeters(35.5) - cubeEdgeHigh;
    public static final double midCubeZ = Units.inchesToMeters(23.5) - cubeEdgeHigh;
    public static final double highConeZ = Units.inchesToMeters(46.0);
    public static final double midConeZ = Units.inchesToMeters(34.0);

    // Translations (all nodes in the same column/row have the same X/Y coordinate)
    public static final Translation2d[] lowTranslations = new Translation2d[nodeRowCount];
    public static final Translation3d[] low3dTranslations = new Translation3d[nodeRowCount];
    public static final Translation2d[] midTranslations = new Translation2d[nodeRowCount];
    public static final Translation3d[] mid3dTranslations = new Translation3d[nodeRowCount];
    public static final Translation2d[] highTranslations = new Translation2d[nodeRowCount];
    public static final Translation3d[] high3dTranslations = new Translation3d[nodeRowCount];

    static {
      for (int i = 0; i < nodeRowCount; i++) {
        boolean isCube = i == 1 || i == 4 || i == 7;
        lowTranslations[i] = new Translation2d(lowX, nodeY[i]);
        low3dTranslations[i] = new Translation3d(lowX, nodeY[i], 0.0);
        midTranslations[i] = new Translation2d(midX, nodeY[i]);
        mid3dTranslations[i] = new Translation3d(midX, nodeY[i], isCube ? midCubeZ : midConeZ);
        highTranslations[i] = new Translation2d(highX, nodeY[i]);
        high3dTranslations[i] = new Translation3d(highX, nodeY[i], isCube ? highCubeZ : highConeZ);
      }
    }

    // Complex low layout (shifted to account for cube vs cone rows and wide edge nodes)
    public static final double complexLowXCones = outerX - Units.inchesToMeters(16.0) / 2.0; // Centered X under cone nodes
    public static final double complexLowXCubes = lowX; // Centered X under cube nodes
    public static final double complexLowOuterYOffset = nodeY[0]
        - (Units.inchesToMeters(3.0) + (Units.inchesToMeters(25.75) / 2.0));

    public static final Translation2d[] complexLowTranslations = new Translation2d[] {
        new Translation2d(complexLowXCones, nodeY[0] - complexLowOuterYOffset),
        new Translation2d(complexLowXCubes, nodeY[1]),
        new Translation2d(complexLowXCones, nodeY[2]),
        new Translation2d(complexLowXCones, nodeY[3]),
        new Translation2d(complexLowXCubes, nodeY[4]),
        new Translation2d(complexLowXCones, nodeY[5]),
        new Translation2d(complexLowXCones, nodeY[6]),
        new Translation2d(complexLowXCubes, nodeY[7]),
        new Translation2d(complexLowXCones, nodeY[8] + complexLowOuterYOffset),
    };

    public static final Translation3d[] complexLow3dTranslations = new Translation3d[] {
        new Translation3d(complexLowXCones, nodeY[0] - complexLowOuterYOffset, 0.0),
        new Translation3d(complexLowXCubes, nodeY[1], 0.0),
        new Translation3d(complexLowXCones, nodeY[2], 0.0),
        new Translation3d(complexLowXCones, nodeY[3], 0.0),
        new Translation3d(complexLowXCubes, nodeY[4], 0.0),
        new Translation3d(complexLowXCones, nodeY[5], 0.0),
        new Translation3d(complexLowXCones, nodeY[6], 0.0),
        new Translation3d(complexLowXCubes, nodeY[7], 0.0),
        new Translation3d(complexLowXCones, nodeY[8] + complexLowOuterYOffset, 0.0),
    };
  }

  // Dimensions for loading zone and substations, including the tape
  public static final class LoadingZone {
    // Region dimensions
    public static final double width = Units.inchesToMeters(99.0);
    public static final double innerX = FieldConstants.fieldLength;
    public static final double midX = fieldLength - Units.inchesToMeters(132.25);
    public static final double outerX = fieldLength - Units.inchesToMeters(264.25);
    public static final double leftY = FieldConstants.fieldWidth;
    public static final double midY = leftY - Units.inchesToMeters(50.5);
    public static final double rightY = leftY - width;
    public static final Translation2d[] regionCorners = new Translation2d[] {
        new Translation2d(
            midX, rightY), // Start at lower left next to border with opponent community
        new Translation2d(midX, midY),
        new Translation2d(outerX, midY),
        new Translation2d(outerX, leftY),
        new Translation2d(innerX, leftY),
        new Translation2d(innerX, rightY),
    };

    // Double substation dimensions
    public static final double doubleSubstationLength = Units.inchesToMeters(14.0);
    public static final double doubleSubstationX = innerX - doubleSubstationLength;
    public static final double doubleSubstationShelfZ = Units.inchesToMeters(37.375);
    public static final double doubleSubstationCenterY = fieldWidth - Units.inchesToMeters(49.76);

    // Single substation dimensions
    public static final double singleSubstationWidth = Units.inchesToMeters(22.75);
    public static final double singleSubstationLeftX = FieldConstants.fieldLength - doubleSubstationLength
        - Units.inchesToMeters(88.77);
    public static final double singleSubstationCenterX = singleSubstationLeftX + (singleSubstationWidth / 2.0);
    public static final double singleSubstationRightX = singleSubstationLeftX + singleSubstationWidth;
    public static final Translation2d singleSubstationTranslation = new Translation2d(singleSubstationCenterX, leftY);

    public static final double singleSubstationHeight = Units.inchesToMeters(18.0);
    public static final double singleSubstationLowZ = Units.inchesToMeters(27.125);
    public static final double singleSubstationCenterZ = singleSubstationLowZ + (singleSubstationHeight / 2.0);
    public static final double singleSubstationHighZ = singleSubstationLowZ + singleSubstationHeight;
  }

  // Locations of staged game pieces
  public static final class StagingLocations {
    public static final double centerOffsetX = Units.inchesToMeters(47.36);
    public static final double positionX = fieldLength / 2.0 - Units.inchesToMeters(47.36);
    public static final double firstY = Units.inchesToMeters(36.19);
    public static final double separationY = Units.inchesToMeters(48.0);
    public static final Translation2d[] translations = new Translation2d[4];

    static {
      for (int i = 0; i < translations.length; i++) {
        translations[i] = new Translation2d(positionX, firstY + (i * separationY));
      }
    }
  }

  // AprilTag constants
  public static final double aprilTagWidth = Units.inchesToMeters(6.5);
  // public AprilTagFieldLayout(String path) throws IOException {
  //   this(Path.of(path));
  // }
  // Unhandled exception type IOExceptionJava(16777384)
  //  handle the exception with java 17

  public static final AprilTagFieldLayout aprilTags = isWPIField ? new AprilTagFieldLayout(
      List.of(
          new AprilTag(
              1,
              new Pose3d(
                  Units.inchesToMeters(610.125),
                  Units.inchesToMeters(43.5),
                  Units.inchesToMeters(19.25),
                  new Rotation3d(0.0, 0.0, Math.PI))),
          new AprilTag(
              2,
              new Pose3d(
                  Units.inchesToMeters(610.375),
                  Units.inchesToMeters(109.5),
                  Units.inchesToMeters(19.25),
                  new Rotation3d(0.0, 0.0, Math.PI))),
          new AprilTag(
              3,
              new Pose3d(
                  Units.inchesToMeters(610.0),
                  Units.inchesToMeters(176.0),
                  Units.inchesToMeters(19.25),
                  new Rotation3d(0.0, 0.0, Math.PI))),
          new AprilTag(
              4,
              new Pose3d(
                  Units.inchesToMeters(635.375),
                  Units.inchesToMeters(272.0),
                  Units.inchesToMeters(27.25),
                  new Rotation3d(0.0, 0.0, Math.PI))),
          new AprilTag(
              5,
              new Pose3d(
                  Units.inchesToMeters(14.25),
                  LoadingZone.doubleSubstationCenterY,
                  Units.inchesToMeters(27.38),
                  new Rotation3d())),
          new AprilTag(
              6,
              new Pose3d(
                  Units.inchesToMeters(40.45),
                  Grids.nodeY[7],
                  Units.inchesToMeters(18.22),
                  new Rotation3d())),
          new AprilTag(
              7,
              new Pose3d(
                  Units.inchesToMeters(40.45),
                  Grids.nodeY[4],
                  Units.inchesToMeters(18.22),
                  new Rotation3d())),
          new AprilTag(
              8,
              new Pose3d(
                  Units.inchesToMeters(40.45),
                  Grids.nodeY[1],
                  Units.inchesToMeters(18.22),
                  new Rotation3d()))),
      fieldLength,
      fieldWidth)
      : new AprilTagFieldLayout(
          List.of(
              new AprilTag(
                  1,
                  new Pose3d(
                      Units.inchesToMeters(610.77),
                      Grids.nodeY[1],
                      Units.inchesToMeters(18.22),
                      new Rotation3d(0.0, 0.0, Math.PI))),
              new AprilTag(
                  2,
                  new Pose3d(
                      Units.inchesToMeters(610.77),
                      Grids.nodeY[4],
                      Units.inchesToMeters(18.22),
                      new Rotation3d(0.0, 0.0, Math.PI))),
              new AprilTag(
                  3,
                  new Pose3d(
                      Units.inchesToMeters(610.77),
                      Grids.nodeY[7],
                      Units.inchesToMeters(18.22),
                      new Rotation3d(0.0, 0.0, Math.PI))),
              new AprilTag(
                  4,
                  new Pose3d(
                      Units.inchesToMeters(636.96),
                      LoadingZone.doubleSubstationCenterY,
                      Units.inchesToMeters(27.38),
                      new Rotation3d(0.0, 0.0, Math.PI))),
              new AprilTag(
                  5,
                  new Pose3d(
                      Units.inchesToMeters(14.25),
                      LoadingZone.doubleSubstationCenterY,
                      Units.inchesToMeters(27.38),
                      new Rotation3d())),
              new AprilTag(
                  6,
                  new Pose3d(
                      Units.inchesToMeters(40.45),
                      Grids.nodeY[7],
                      Units.inchesToMeters(18.22),
                      new Rotation3d())),
              new AprilTag(
                  7,
                  new Pose3d(
                      Units.inchesToMeters(40.45),
                      Grids.nodeY[4],
                      Units.inchesToMeters(18.22),
                      new Rotation3d())),
              new AprilTag(
                  8,
                  new Pose3d(
                      Units.inchesToMeters(40.45),
                      Grids.nodeY[1],
                      Units.inchesToMeters(18.22),
                      new Rotation3d()))),
          fieldLength,
          fieldWidth);

  public static AprilTagFieldLayout getAprilTags() {
    String jsonString = "{\"tags\":[{\"ID\":1,\"pose\":{\"translation\":{\"x\":15.079471999999997,\"y\":0.24587199999999998,\"z\":1.355852},\"rotation\":{\"quaternion\":{\"W\":0.5000000000000001,\"X\":0.0,\"Y\":0.0,\"Z\":0.8660254037844386}}}},{\"ID\":2,\"pose\":{\"translation\":{\"x\":16.185134,\"y\":0.883666,\"z\":1.355852},\"rotation\":{\"quaternion\":{\"W\":0.5000000000000001,\"X\":0.0,\"Y\":0.0,\"Z\":0.8660254037844386}}}},{\"ID\":3,\"pose\":{\"translation\":{\"x\":16.579342,\"y\":4.982717999999999,\"z\":1.4511020000000001},\"rotation\":{\"quaternion\":{\"W\":6.123233995736766E-17,\"X\":0.0,\"Y\":0.0,\"Z\":1.0}}}},{\"ID\":4,\"pose\":{\"translation\":{\"x\":16.579342,\"y\":5.547867999999999,\"z\":1.4511020000000001},\"rotation\":{\"quaternion\":{\"W\":6.123233995736766E-17,\"X\":0.0,\"Y\":0.0,\"Z\":1.0}}}},{\"ID\":5,\"pose\":{\"translation\":{\"x\":14.700757999999999,\"y\":8.2042,\"z\":1.355852},\"rotation\":{\"quaternion\":{\"W\":-0.7071067811865475,\"X\":-0.0,\"Y\":0.0,\"Z\":0.7071067811865476}}}},{\"ID\":6,\"pose\":{\"translation\":{\"x\":1.8415,\"y\":8.2042,\"z\":1.355852},\"rotation\":{\"quaternion\":{\"W\":-0.7071067811865475,\"X\":-0.0,\"Y\":0.0,\"Z\":0.7071067811865476}}}},{\"ID\":7,\"pose\":{\"translation\":{\"x\":-0.038099999999999995,\"y\":5.547867999999999,\"z\":1.4511020000000001},\"rotation\":{\"quaternion\":{\"W\":1.0,\"X\":0.0,\"Y\":0.0,\"Z\":0.0}}}},{\"ID\":8,\"pose\":{\"translation\":{\"x\":-0.038099999999999995,\"y\":4.982717999999999,\"z\":1.4511020000000001},\"rotation\":{\"quaternion\":{\"W\":1.0,\"X\":0.0,\"Y\":0.0,\"Z\":0.0}}}},{\"ID\":9,\"pose\":{\"translation\":{\"x\":0.356108,\"y\":0.883666,\"z\":1.355852},\"rotation\":{\"quaternion\":{\"W\":0.8660254037844387,\"X\":0.0,\"Y\":0.0,\"Z\":0.49999999999999994}}}},{\"ID\":10,\"pose\":{\"translation\":{\"x\":1.4615159999999998,\"y\":0.24587199999999998,\"z\":1.355852},\"rotation\":{\"quaternion\":{\"W\":0.8660254037844387,\"X\":0.0,\"Y\":0.0,\"Z\":0.49999999999999994}}}},{\"ID\":11,\"pose\":{\"translation\":{\"x\":11.904726,\"y\":3.7132259999999997,\"z\":1.3208},\"rotation\":{\"quaternion\":{\"W\":-0.8660254037844387,\"X\":-0.0,\"Y\":0.0,\"Z\":0.49999999999999994}}}},{\"ID\":12,\"pose\":{\"translation\":{\"x\":11.904726,\"y\":4.49834,\"z\":1.3208},\"rotation\":{\"quaternion\":{\"W\":0.8660254037844387,\"X\":0.0,\"Y\":0.0,\"Z\":0.49999999999999994}}}},{\"ID\":13,\"pose\":{\"translation\":{\"x\":11.220196,\"y\":4.105148,\"z\":1.3208},\"rotation\":{\"quaternion\":{\"W\":6.123233995736766E-17,\"X\":0.0,\"Y\":0.0,\"Z\":1.0}}}},{\"ID\":14,\"pose\":{\"translation\":{\"x\":5.320792,\"y\":4.105148,\"z\":1.3208},\"rotation\":{\"quaternion\":{\"W\":1.0,\"X\":0.0,\"Y\":0.0,\"Z\":0.0}}}},{\"ID\":15,\"pose\":{\"translation\":{\"x\":4.641342,\"y\":4.49834,\"z\":1.3208},\"rotation\":{\"quaternion\":{\"W\":0.5000000000000001,\"X\":0.0,\"Y\":0.0,\"Z\":0.8660254037844386}}}},{\"ID\":16,\"pose\":{\"translation\":{\"x\":4.641342,\"y\":3.7132259999999997,\"z\":1.3208},\"rotation\":{\"quaternion\":{\"W\":-0.49999999999999983,\"X\":-0.0,\"Y\":0.0,\"Z\":0.8660254037844388}}}}],\"field\":{\"length\":16.451,\"width\":8.211}}";
    AprilTagFieldLayout aprilTags = new AprilTagFieldLayout(
        List.of(
            new AprilTag(
                1,
                new Pose3d(
                    15.079471999999997,
                    0.24587199999999998,
                    1.355852,
                    new Rotation3d(new Quaternion(0.5000000000000001, 0.0, 0.0, 0.8660254037844386)))),
            new AprilTag(
                2,
                new Pose3d(
                    16.185134,
                    0.883666,
                    1.355852,
                    new Rotation3d(new Quaternion(0.5000000000000001, 0.0, 0.0, 0.8660254037844386)))),
            new AprilTag(
                3,
                new Pose3d(
                    16.579342,
                    4.982717999999999,
                    1.4511020000000001,
                    new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 1.0)))),
            new AprilTag(
                4,
                new Pose3d(
                    16.579342,
                    5.547867999999999,
                    1.4511020000000001,
                    new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 1.0)))),
            new AprilTag(
                5,
                new Pose3d(
                    14.700757999999999,
                    8.2042,
                    1.355852,
                    new Rotation3d(new Quaternion(-0.7071067811865475, 0.0, 0.0, 0.7071067811865476)))),
            new AprilTag(
                6,
                new Pose3d(
                    1.8415,
                    8.2042,
                    1.355852,
                    new Rotation3d(new Quaternion(-0.7071067811865475, 0.0, 0.0, 0.7071067811865476)))),
            new AprilTag(
                7,
                new Pose3d(
                    -0.038099999999999995,
                    5.547867999999999,
                    1.4511020000000001,
                    new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
            new AprilTag(
                8,
                new Pose3d(
                    -0.038099999999999995,
                    4.982717999999999,
                    1.4511020000000001,
                    new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
            new AprilTag(
                9,
                new Pose3d(
                    0.356108,
                    0.883666,
                    1.355852,
                    new Rotation3d(new Quaternion(0.8660254037844387, 0.0, 0.0, 0.49999999999999994)))),
            new AprilTag(
                10,
                new Pose3d(
                    1.4615159999999998,
                    0.24587199999999998,
                    1.355852,
                    new Rotation3d(new Quaternion(0.8660254037844387, 0.0, 0.0, 0.49999999999999994)))),
            new AprilTag(
                11,
                new Pose3d(
                    11.904726,
                    3.7132259999999997,
                    1.3208,
                    new Rotation3d(new Quaternion(-0.8660254037844387, 0.0, 0.0, 0.49999999999999994)))),
            new AprilTag(
                12,
                new Pose3d(
                    11.904726,
                    4.49834,
                    1.3208,
                    new Rotation3d(new Quaternion(0.8660254037844387, 0.0, 0.0, 0.49999999999999994)))),
            new AprilTag(
                13,
                new Pose3d(
                    11.220196,
                    4.105148,
                    1.3208,
                    new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 1.0)))),
            new AprilTag(
                14,
                new Pose3d(
                    5.320792,
                    4.105148,
                    1.3208,
                    new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
            new AprilTag(
                15,
                new Pose3d(
                    4.641342,
                    4.49834,
                    1.3208,
                    new Rotation3d(new Quaternion(0.5000000000000001, 0.0, 0.0, 0.8660254037844386)))),
            new AprilTag(
                16,
                new Pose3d(
                    4.641342,
                    3.7132259999999997,
                    1.3208,
                    new Rotation3d(new Quaternion(-0.49999999999999983, 0.0, 0.0, 0.8660254037844388))))),
        fieldLength, fieldWidth);

    // AprilTagFieldLayout aprilTags = new AprilTagFieldLayout(
    //     List.of(
    //         new AprilTag(
    //             1,
    //             new Pose3d(
    //                 Units.inchesToMeters(610.125),
    //                 Units.inchesToMeters(43.5),
    //                 Units.inchesToMeters(19.25),
    //                 new Rotation3d(0.0, 0.0, Math.PI))),
    //         new AprilTag(
    //             2,
    //             new Pose3d(
    //                 Units.inchesToMeters(610.375),
    //                 Units.inchesToMeters(109.5),
    //                 Units.inchesToMeters(19.25),
    //                 new Rotation3d(0.0, 0.0, Math.PI))),
    //         new AprilTag(
    //             3,
    //             new Pose3d(
    //                 Units.inchesToMeters(610.0),
    //                 Units.inchesToMeters(176.0),
    //                 Units.inchesToMeters(19.25),
    //                 new Rotation3d(0.0, 0.0, Math.PI))),
    //         new AprilTag(
    //             4,
    //             new Pose3d(
    //                 Units.inchesToMeters(635.375),
    //                 Units.inchesToMeters(272.0),
    //                 Units.inchesToMeters(27.25),
    //                 new Rotation3d(0.0, 0.0, Math.PI))),
    //         new AprilTag(
    //             5,
    //             new Pose3d(
    //                 Units.inchesToMeters(14.25),
    //                 LoadingZone.doubleSubstationCenterY,
    //                 Units.inchesToMeters(27.38),
    //                 new Rotation3d())),
    //         new AprilTag(
    //             6,
    //             new Pose3d(
    //                 Units.inchesToMeters(40.45),
    //                 Grids.nodeY[7],
    //                 Units.inchesToMeters(18.22),
    //                 new Rotation3d())),
    //         new AprilTag(
    //             7,
    //             new Pose3d(
    //                 Units.inchesToMeters(40.45),
    //                 Grids.nodeY[4],
    //                 Units.inchesToMeters(18.22),
    //                 new Rotation3d())),
    //         new AprilTag(
    //             8,
    //             new Pose3d(
    //                 Units.inchesToMeters(40.45),
    //                 Grids.nodeY[1],
    //                 Units.inchesToMeters(18.22),
    //                 new Rotation3d()))),
    //     fieldLength,
    //     fieldWidth);

    return aprilTags;
  }
}
