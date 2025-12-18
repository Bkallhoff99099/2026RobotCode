// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LimelightHelpers;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private String m_name;
  private SwerveDrivePoseEstimator m_poseEstimator;
  private AHRS m_gyro;
  private boolean doRejectUpdate;

  public Vision(String name, SwerveDrivePoseEstimator poseEstimator, AHRS gyro, double[] config) {
    m_name = name;
    m_poseEstimator = poseEstimator;
    m_gyro = gyro;


    LimelightHelpers.setCameraPose_RobotSpace(name, config[0], config[1], config[2], config[3], config[4], config[5]);
  }

  /**
   * drives to the current apriltag
   * @return the speed to drive at
   */
  public double rangeWithVision(){
    double kP = 0.1;
    double targetingForwardSpeed = LimelightHelpers.getTY(m_name) * kP;
    targetingForwardSpeed *= 4; // 4 m/s is the max speed for the drivetrain
    return targetingForwardSpeed;
  }

  /**
   * drives to a distance away from the current aprilTag
   * @param distance the distance in meters you want to be from the target
   * @return the speed to drive at
   */
  public double driveToRangeWithVision(double distance){
    double kP = 0.1;
    double targetingForwardSpeed = (LimelightHelpers.getTY(m_name) - distance) * kP;
    targetingForwardSpeed *= 4;
    return targetingForwardSpeed;
  }

  /**
   * aims at the current aprilTag
   */
  public double aimWithVision(){
    double kP = 0.0017;
    double targetingAngularVelocity = LimelightHelpers.getTX(m_name) * kP;
    targetingAngularVelocity *= 4; // the max angular velocity
    targetingAngularVelocity *= -1;
    return targetingAngularVelocity;
  }

  public Pose2d getPose(){
    return LimelightHelpers.getBotPose2d_wpiBlue(m_name);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    doRejectUpdate = false;
    LimelightHelpers.SetRobotOrientation(m_name,
     m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
      0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate megaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_name);
    if(Math.abs(m_gyro.getRate()) > 360){
      doRejectUpdate = true;
    }
    if(megaTag2.tagCount == 0){
      doRejectUpdate = true;
    }
    if(!doRejectUpdate){
      m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7, 9999999));
      m_poseEstimator.addVisionMeasurement(megaTag2.pose, megaTag2.timestampSeconds);
    }
  }
}
