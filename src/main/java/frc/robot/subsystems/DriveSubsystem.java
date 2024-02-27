// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.controls.DutyCycleOut;

import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.AutoConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import com.pathplanner.lib.auto.AutoBuilder;

// import com.pathplanner.lib.auto.AutoBuilder.*;
// import com.pathplanner.lib.commands.FollowPathHolonomic;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PathPlannerLogging;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
// import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Map;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriverConstants.kFrontLeftDrivingCanId,
      DriverConstants.kFrontLeftTurningCanId,
      DriverConstants.kFrontLeftChassisAngularOffset,
      DriverConstants.kFrontLeftCanCoderId,
      DriverConstants.kFrontLeftDrivingInvert,
      DriverConstants.kFrontLeftTurningInvert);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriverConstants.kFrontRightDrivingCanId,
      DriverConstants.kFrontRightTurningCanId,
      DriverConstants.kFrontRightChassisAngularOffset,
      DriverConstants.kFrontRightCanCoderId,
      DriverConstants.kFrontRightDrivingInvert,
      DriverConstants.kFrontRightTurningInvert);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriverConstants.kRearLeftDrivingCanId,
      DriverConstants.kRearLeftTurningCanId,
      DriverConstants.kBackLeftChassisAngularOffset,
      DriverConstants.kRearLeftCanCoderId,
      DriverConstants.kRearLeftDrivingInvert,
      DriverConstants.kRearLeftTurningInvert);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriverConstants.kRearRightDrivingCanId,
      DriverConstants.kRearRightTurningCanId,
      DriverConstants.kBackRightChassisAngularOffset,
      DriverConstants.kRearRightCanCoderId,
      DriverConstants.kRearRightDrivingInvert,
      DriverConstants.kRearRightTurningInvert);

  // The gyro sensor
  private final Pigeon2 m_gyro = new Pigeon2(0);
  // private final DutyCycleOut control = new DutyCycleOut(getHeading());
  private Field2d field = new Field2d();

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriverConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriverConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriverConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    // PIDConstants thetaController = new PIDConstants(
    // AutoConstants.kPThetaController, 0, 0,
    // AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // AutoBuilder.configureHolonomic(
    // this::getPose,
    // this::resetOdometry,
    // this::getCurrentSpeeds,
    // this::driveRobotRelative,
    // new HolonomicPathFollowerConfig(

    // new PIDConstants(3, 0, 0.00),
    // // 断点：设置 运动和旋转PId
    // new PIDConstants(3, 0, 0.0),
    // AutoConstants.kMaxSpeedMetersPerSecond,//
    // 0.40299,//中心到模组半径— （0.57/2） * 根号2(适用于正方形底盘！)

    // new ReplanningConfig()),
    // ()->{
    // var alliance = DriverStation.getAlliance();
    // if (alliance.isPresent()){
    // return alliance.get() == DriverStation.Alliance.Blue;//Red 红队
    // }
    // return false;
    // },

    // this);

    // // Set up custom logging to add the current path to a field 2d widget
    // PathPlannerLogging.setLogActivePathCallback((poses) ->
    // field.getObject("path").setPoses(poses));

    // SmartDashboard.putData("Field", field);
    zeroHeading();
  }

  public void DEBUG_printTurnEncoders() {
    System.out.println(
        m_frontLeft.getPosition() + " " +
            m_frontRight.getPosition() + " " +
            m_rearLeft.getPosition() + " " +
            m_rearRight.getPosition());
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block

    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    field.setRobotPose(getPose());
    // DEBUG_printTurnEncoders();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  // 来自3015的speed和relativespeed
  /**
   * Drive the robot with field relative chassis speeds
   *
   * @param fieldRelativeSpeeds Field relative chassis speeds
   */
  //
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    };
    return states;
  }

  public ChassisSpeeds getCurrentSpeeds() {
    return DriverConstants.kDriveKinematics.toChassisSpeeds(getStates());
  }

  // 根据速度设定去运动！
  public void driveRobotRelative(ChassisSpeeds speeds) {
    SwerveModuleState[] targetStates = DriverConstants.kDriveKinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, AutoConstants.kMaxSpeedMetersPerSecond);

    setModuleStates(targetStates);
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        fieldRelativeSpeeds.vxMetersPerSecond,
        fieldRelativeSpeeds.vyMetersPerSecond,
        fieldRelativeSpeeds.omegaRadiansPerSecond,
        Rotation2d.fromDegrees(m_gyro.getAngle())
    // getPose().getRotation()
    );

    driveRobotRelative(speeds);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed); // 计算xy夹角弧度
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));// 计算斜边

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriverConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;

    
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
  
      if (angleDif < 0.025 * Math.PI) {
 
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.875 * Math.PI) {
        if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir); // x 速度分量是总速度的cos
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);// y 速度分量是总速度的sin
      m_currentRotation = m_rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriverConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriverConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriverConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriverConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(m_gyro.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriverConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);

  }

  // public ChassisSpeeds getRobotRelativeSpeeds(){
  // return ChassisSpeeds.fromFieldRelativeSpeeds(getRobotRelativeSpeeds(), null)
  // }
  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));// -45
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));// -45
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriverConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);

  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.setYaw(0);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriverConstants.kGyroReversed ? -1.0 : 1.0);
  }

}
