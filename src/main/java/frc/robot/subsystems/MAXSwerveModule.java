// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
// import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
// import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import javax.tools.Diagnostic;

import com.ctre.phoenix6.hardware.CANcoder;

import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.ModuleConstants;

public class MAXSwerveModule {
  private final CANSparkMax m_drivingSparkMax;
  private final CANSparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final CANcoder absoluteEncoder;
  private final RelativeEncoder m_turningEncoder;

  private final SparkPIDController m_drivingPIDController;
  private final SparkPIDController m_turningPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset, int CanCoderID,
      boolean DrivingInvert, boolean TurningInvert) {//
    m_drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_drivingSparkMax.restoreFactoryDefaults();
    m_turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    absoluteEncoder = new CANcoder(CanCoderID);
    m_turningEncoder = m_turningSparkMax.getEncoder();

    m_drivingPIDController = m_drivingSparkMax.getPIDController();
    m_turningPIDController = m_turningSparkMax.getPIDController();
    m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    m_drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
    m_drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);
    m_turningEncoder.setPositionConversionFactor(0.2932153);// 7/150*6.28
    m_turningEncoder.setVelocityConversionFactor(0.00488692182);// 7/150*2*3.14/60

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    m_drivingEncoder.setPosition(0);
    m_turningEncoder
        .setPosition(-absoluteEncoder.getAbsolutePosition().getValue() * Math.PI * 2 + m_chassisAngularOffset);
    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // // the steering motor in the MAXSwerve Module.
    // m_turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and
    // you
    // may need to tune them for your own robot!
    m_drivingPIDController.setP(ModuleConstants.kDrivingP);
    m_drivingPIDController.setI(ModuleConstants.kDrivingI);
    m_drivingPIDController.setD(ModuleConstants.kDrivingD);
    m_drivingPIDController.setFF(ModuleConstants.kDrivingFF);
    m_drivingPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput,
        ModuleConstants.kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and
    // you
    // may need to tune them for your own robot!
    m_turningPIDController.setP(ModuleConstants.kTurningP);
    m_turningPIDController.setI(ModuleConstants.kTurningI);
    m_turningPIDController.setD(ModuleConstants.kTurningD);
    m_turningPIDController.setFF(ModuleConstants.kTurningFF);
    m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);

    m_drivingSparkMax.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
    m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    m_drivingSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);
    m_drivingSparkMax.setInverted(DrivingInvert);
    m_turningSparkMax.setInverted(TurningInvert);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_drivingSparkMax.burnFlash();
    m_turningSparkMax.burnFlash();

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());

  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.

    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(-absoluteEncoder.getAbsolutePosition().getValue() * Math.PI * 2 + m_chassisAngularOffset));
  }

  private int encoderWindUp = 0;
  private double prevAngle = 0.0;

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {

    if (Math.abs(desiredState.speedMetersPerSecond) < 0.01) {
      desiredState.speedMetersPerSecond = 0;
    }

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState state = SwerveModuleState.optimize(desiredState,
        new Rotation2d(m_turningEncoder.getPosition()));

    // System.out.println("abs: " + (int) (-absoluteEncoder.getAbsolutePosition().getValue() * 360) +
    //     "\tabs_cor: "
    //     + (int) (-absoluteEncoder.getAbsolutePosition().getValue() * 360 + m_chassisAngularOffset / Math.PI / 2 * 360)
    //     + "\topt: " + (int) (state.angle.getDegrees())
    //     + "\treal: " + (int) (m_turningEncoder.getPosition() / Math.PI * 180));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    // DEBUG: Line*2
    m_drivingPIDController.setReference(state.speedMetersPerSecond,
        CANSparkMax.ControlType.kVelocity);

    // New Code added to handle the 180 to -180 and -180 t0 180 boundary condition
    if (prevAngle > 90) {
      // Our previous angle was in an area where a wrap could oocur (SW Corner)
      if (state.angle.getDegrees() < -90) {
        // If we are here it means that target setpoint needs to be adjusted to prevent
        // the
        // module from taking the long way to reach the target in the SE Corner.
        encoderWindUp++;
      }
    }

    if (prevAngle < -90) {
      // Our previous angle was in an area where a wrap could occur (SE Corner)
      if (state.angle.getDegrees() > 90) {
        // If we are here it mean that target setpoint needs to be adjusted to prevent
        // the
        // module from taking the long way to reach the target in the SW Corner.
        encoderWindUp--;
      }
    }

    // Update the last setpoint to the current setpoint
    prevAngle = state.angle.getDegrees();

    m_turningPIDController.setReference(state.angle.getRadians() + encoderWindUp * 2 * Math.PI,
        CANSparkMax.ControlType.kPosition);

    // System.out.println(optimizedDesiredState.angle.getRadians());
    // m_desiredState = optimizedDesiredState;//desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
    // m_turningEncoder.setPosition(absoluteEncoder.getAbsolutePosition().getValue()
    // * Math.PI*2);
    m_turningEncoder
        .setPosition(-absoluteEncoder.getAbsolutePosition().getValue() * Math.PI * 2 + m_chassisAngularOffset);
  }
}
