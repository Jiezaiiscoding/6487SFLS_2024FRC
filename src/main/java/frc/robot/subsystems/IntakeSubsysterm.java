// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsysterm extends SubsystemBase {
  private  CANSparkMax m_intake = new CANSparkMax(21,MotorType.kBrushless);
    // private final SparkPIDController intakPidController = m_intake.getPIDController();
    // private final RelativeEncoder intakEncoder = m_intake.getEncoder();
    // public static boolean inited = false;

   
  public IntakeSubsysterm() {
    m_intake.restoreFactoryDefaults();
    m_intake.setInverted(true);

        // inited = true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set_intake(double speed){
    m_intake.set(speed);
  }

  public void stopMotor(){
    m_intake.set(0);
  }



  public double get_intake(){
    return m_intake.get();
  }
}
