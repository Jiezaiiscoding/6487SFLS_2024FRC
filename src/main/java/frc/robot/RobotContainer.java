// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

import frc.robot.subsystems.IntakeSubsysterm;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import java.util.HashMap;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.Pigeon2;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.path.GoalEndState;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.util.Units;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  private final IntakeSubsysterm m_IntakeSubsysterm = new IntakeSubsysterm();

  // The driver's controller
  public XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  // 选择命令
  // private final SendableChooser<Command> m_Chooser =
  // AutoBuilder.buildAutoChooser();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
    // NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
    // NamedCommands.registerCommand("print hello", Commands.print("hello"));

    m_robotDrive.resetEncoders();

    // Configure the button bindings
    configureButtonBindings();

    // pathplanner的chooser
    // SmartDashboard.putData("Auto Mode", m_Chooser);
    // }
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.

        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(m_driverController.getLeftY()*0.5, OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(-m_driverController.getLeftX()*0.5, OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(-m_driverController.getRightX()*0.5, OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, 1).whileTrue(new RunCommand(() -> {
      m_robotDrive.zeroHeading();
    }));
    new JoystickButton(m_driverController, 2).whileTrue(new RunCommand(() -> {
      m_robotDrive.resetEncoders();
    }));

    // m_driverController.getAButton(m_gyro.reset());

    // new JoystickButton(m_driverController, 2).onTrue(
    // new InstantCommand(()->{
    // m_IntakeSubsysterm.set_intake(m_driverController.getRawAxis(3));
    // },m_IntakeSubsysterm

    // ));
    // if (m_driverController.getRawAxis(3)>0.15){
    // m_IntakeSubsysterm.set_intake(m_driverController.getRawAxis(3));
    // }
    // m_IntakeSubsysterm.set_intake(0.8);

    // Add a button to run the example auto to SmartDashboard, this will also be in
    // the auto chooser built above
    // SmartDashboard.putData("AutoPath", new PathPlannerAuto("AutoPath"));

    // Add a button to run pathfinding commands to SmartDashboard

    // SmartDashboard.putData("Pathfind to Pickup Pos", AutoBuilder.pathfindToPose(
    // new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)),
    // new PathConstraints(
    // 4.0, 4.0,
    // Units.degreesToRadians(360), Units.degreesToRadians(540)
    // ),
    // 0,
    // 2.0
    // ));

    // SmartDashboard.putData("Pathfind to Scoring Pos", AutoBuilder.pathfindToPose(
    // new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)),
    // new PathConstraints(
    // 4.0, 4.0,
    // Units.degreesToRadians(360), Units.degreesToRadians(540)
    // ),
    // 0,
    // 0
    // ));

    // Add a button to SmartDashboard that will create and follow an on-the-fly path
    // This example will simply move the robot 2m in the +X field direction
    // SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {
    // Pose2d currentPose = m_robotDrive.getPose();

    // // The rotation component in these poses represents the direction of travel
    // Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
    // Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new
    // Translation2d(2.0, 0.0)), new Rotation2d());
    // 不限两个加多少都可以
    // List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos,
    // endPos);
    // PathPlannerPath path = new PathPlannerPath(
    // bezierPoints,
    // new PathConstraints(
    // 4.0, 4.0,
    // Units.degreesToRadians(360), Units.degreesToRadians(540)
    // ),
    // new GoalEndState(0.0, currentPose.getRotation())
    // );

    // // Prevent this path from being flipped on the red alliance, since the given
    // positions are already correct
    // path.preventFlipping = true;

    // AutoBuilder.followPath(path).schedule();
    // }));

    // }

    // /**
    // * Use this to pass the autonomous command to the main {@link Robot} class.
    // *
    // * @return the command to run in autonomous
    // */
    // public Command getAutonomousCommand() {

    // //方案4 多条线路
    // return m_Chooser.getSelected();

  }
}
