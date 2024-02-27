// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PowerDistribution;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;





public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  

  private final PowerDistribution m_PDH = new PowerDistribution();

  private CANcoder FL_cancoder = new CANcoder(0);
  private CANcoder FR_cancoder = new CANcoder(1);
  private CANcoder BR_cancoder = new CANcoder(2);
  private CANcoder BL_cancoder = new CANcoder(3);

  private final Pigeon2 m_gyro = new Pigeon2(0);
  public XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putData(m_PDH);
    SmartDashboard.putNumber("cancoder0", FL_cancoder.getAbsolutePosition().getValue() * Math.PI*2);
    SmartDashboard.putNumber("cancoder1", FR_cancoder.getAbsolutePosition().getValue() * Math.PI*2);
    SmartDashboard.putNumber("cancoder3", BL_cancoder.getAbsolutePosition().getValue() * Math.PI*2);
    SmartDashboard.putNumber("cancoder2", BR_cancoder.getAbsolutePosition().getValue() * Math.PI*2);

    SmartDashboard.putNumber("gyro_angle", m_gyro.getAngle());

    SmartDashboard.putNumber("left-x", m_driverController.getLeftX()); 
    SmartDashboard.putNumber("left-y", m_driverController.getLeftY()); 

    // SmartDashboard.putNumber("Position", m_Intake.getEncoder().getPosition());
    // SmartDashboard.putNumber("Velocity", m_Intake.getEncoder().getVelocity());
  
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    //sb 下拉choice
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
