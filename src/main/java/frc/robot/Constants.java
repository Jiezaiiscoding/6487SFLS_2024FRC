// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class DriverConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 2.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 2.5; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = 0.55;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.52;
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        // new Translation2d(-kWheelBase / 2, kTrackWidth / 2),// 左前
        // new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),// 右前 --
        // new Translation2d(kWheelBase / 2, kTrackWidth / 2),//左后++
        // new Translation2d(kWheelBase / 2, -kTrackWidth / 2));//右后
        // 按照安装
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // 左前
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), // 右前
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), // 左后
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2));// 右后

    // Angular offsets of the modules relative to the cha%ssis in radians
    // 用于调整偏差:先置零一次，deploy，enable后定头，直行一下，然后disable，把轮子掰直，再读取对应数字
    // public static final double kFrontLeftChassisAngularOffset = 0;//coder0
    // public static final double kFrontRightChassisAngularOffset = 0;//coder1
    // public static final double kBackLeftChassisAngularOffset = 0;//coder3
    // public static final double kBackRightChassisAngularOffset = 0;//coder2

    public static final double kFrontLeftChassisAngularOffset = -0.004601942363657 + Math.PI; // coder0;
    public static final double kFrontRightChassisAngularOffset = 0.756252528427621;// coder1
    public static final double kBackLeftChassisAngularOffset = -1.560058461279697;// coder3
    public static final double kBackRightChassisAngularOffset = 1.271670073157197;// coder2

    // // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 10;
    public static final int kRearLeftDrivingCanId = 16;
    public static final int kFrontRightDrivingCanId = 12;
    public static final int kRearRightDrivingCanId = 14;

    public static final int kFrontLeftTurningCanId = 11;
    public static final int kRearLeftTurningCanId = 17;
    public static final int kFrontRightTurningCanId = 13;
    public static final int kRearRightTurningCanId = 15;

    // CAN coder ID
    public static final int kFrontLeftCanCoderId = 0;
    public static final int kRearLeftCanCoderId = 3;
    public static final int kFrontRightCanCoderId = 1;
    public static final int kRearRightCanCoderId = 2;

    // reverse

    public static final boolean kFrontLeftDrivingInvert = false;
    public static final boolean kFrontRightDrivingInvert = false;
    public static final boolean kRearLeftDrivingInvert = false;
    public static final boolean kRearRightDrivingInvert = false;

    public static final boolean kFrontLeftTurningInvert = false;
    public static final boolean kFrontRightTurningInvert = false;
    public static final boolean kRearLeftTurningInvert = false;
    public static final boolean kRearRightTurningInvert = false;

    public static final boolean CanCoderInvert = false;

    public static final boolean kGyroReversed = false;

  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.10;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    //
    public static final double kDrivingMotorReduction = (45.0 * 17 * 50) / (kDrivingMotorPinionTeeth * 15 * 17);//
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kAngleGearRatio = 7 / 150;
    public static final double kTurningEncoderPositionFactor = kAngleGearRatio * (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = kAngleGearRatio * (2 * Math.PI) / 60.0; // radians per
                                                                                                       // second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.005;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0.0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0;
    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 4;
    public static final int kDriverFieldOrientedButtonIdx = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    // public static final PIDConstants CONSTANTS_X = new PIDConstants(0.1, 0, 0);

    // public static final PIDConstants THETA_CONSTANTS = new PIDConstants(0.01, 0,
    // 0);

  }

  public static final class PathPlannerConstants {
    // public static final PathConstraints constraints = new PathConstraints(2, 1,
    // 0, 0);

  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
