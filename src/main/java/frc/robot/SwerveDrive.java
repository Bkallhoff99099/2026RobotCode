// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
  
  
  SwerveDriveKinematics kinematics;
  SwerveDriveOdometry   odometry;
  AHRS                  gyro; // Psuedo-class representing a gyroscope.
  SwerveModule[]         swerveModules;

  /** Creates a new SwerveModule. */
  public SwerveDrive() {
    swerveModules = new SwerveModule[4];

    // Create SwerveDriveKinematics object
        // 12.5in from center of robot to center of wheel.
        // 12.5in is converted to meters to work with object.
        // Translation2d(x,y) == Translation2d(front, left)
        kinematics = new SwerveDriveKinematics(
            new Translation2d(Units.inchesToMeters(12.5), Units.inchesToMeters(12.5)), // Front Left
            new Translation2d(Units.inchesToMeters(12.5), Units.inchesToMeters(-12.5)), // Front Right
            new Translation2d(Units.inchesToMeters(-12.5), Units.inchesToMeters(12.5)), // Back Left
            new Translation2d(Units.inchesToMeters(-12.5), Units.inchesToMeters(-12.5))  // Back Right
        );
    gyro = new AHRS(NavXComType.kMXP_SPI);

    odometry = new SwerveDriveOdometry(
            kinematics,
            Rotation2d.fromDegrees(gyro.getAngle()), // returns current gyro reading as a Rotation2d
            new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()},
            // Front-Left, Front-Right, Back-Left, Back-Right
            new Pose2d(0,0,new Rotation2d()) // x=0, y=0, heading=0
        );
  }

  public void drive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rot){
    ChassisSpeeds driveSpeed = new ChassisSpeeds(xSpeed.getAsDouble(), ySpeed.getAsDouble(), rot.getAsDouble());
    SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(driveSpeed);

    swerveModules[0].setState(swerveModuleStates[0]);
    swerveModules[1].setState(swerveModuleStates[1]);
    swerveModules[2].setState(swerveModuleStates[2]);
    swerveModules[3].setState(swerveModuleStates[3]);
  }

  public SwerveModulePosition[] getCurrentSwerveModulePositions()
  {
      return new SwerveModulePosition[]{
          new SwerveModulePosition(swerveModules[0].getDistance(), swerveModules[0].getAngle()), // Front-Left
          new SwerveModulePosition(swerveModules[1].getDistance(), swerveModules[1].getAngle()), // Front-Right
          new SwerveModulePosition(swerveModules[2].getDistance(), swerveModules[2].getAngle()), // Back-Left
          new SwerveModulePosition(swerveModules[3].getDistance(), swerveModules[3].getAngle())  // Back-Right
      };
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(Rotation2d.fromDegrees(gyro.getAngle()),  getCurrentSwerveModulePositions());
  }
}
