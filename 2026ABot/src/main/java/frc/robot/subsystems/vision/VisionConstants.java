// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "Camera0"; // hopper
  public static String camera1Name = "Camera1"; // fr
  public static String camera2Name = "Camera2"; // br
  public static String camera3Name = "Camera3"; // bl

  // Robot to camera transforms
  public static Transform3d robotToCamera0 =
      new Transform3d(
          Units.inchesToMeters(7.75),
          Units.inchesToMeters(13.375),
          Units.inchesToMeters(20.25),
          new Rotation3d(0.0, Math.toRadians(-2), Math.toRadians(90)));

  public static Transform3d robotToCamera1 =
      new Transform3d(
          Units.inchesToMeters(7.25),
          Units.inchesToMeters(-12.25),
          Units.inchesToMeters(16.125),
          new Rotation3d(0.0, Math.toRadians(-25), Math.toRadians(315)));
  public static Transform3d robotToCamera2 =
      new Transform3d(
          -0.2831, -0.2903, 0.1923, new Rotation3d(0.0, Math.toRadians(-25), Math.toRadians(225)));
  public static Transform3d robotToCamera3 =
      new Transform3d(
          -0.2831, 0.2883, 0.1923, new Rotation3d(0.0, Math.toRadians(-25), Math.toRadians(135)));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.15;
  public static double maxZError = 0.75;
  public static double maxDistance = 6.0;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
