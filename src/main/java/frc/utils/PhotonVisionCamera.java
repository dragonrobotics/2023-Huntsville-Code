// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;

public class PhotonVisionCamera {
  private PhotonCamera camera;
  private AprilTagFieldLayout aprilTagFieldLayout;
  private Transform3d camPosition;
  private PhotonPoseEstimator photonPoseEstimator;

  public PhotonVisionCamera(String cameraName, Transform3d cameraPosition) {

    camPosition = cameraPosition;
    camera = new PhotonCamera(cameraName);

    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
      photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, camera,
          camPosition);
    } catch (Exception e) {
      DriverStation.reportError("Error creating photonvision camera!", true);
      photonPoseEstimator = null;
    }
  }

  public Optional<EstimatedRobotPose> getPose() {
    if (photonPoseEstimator == null)
      return Optional.empty();
    return photonPoseEstimator.update();
  }

}
