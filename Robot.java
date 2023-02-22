// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Stabilize;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private Stabilize stabilize = new Stabilize();

  private RobotContainer m_robotContainer;
  
  UsbCamera camera1;
  UsbCamera camera2;
  private int cameraNum = 1;
  VideoSink server;
  NetworkTableEntry cameraSelection;
  Joystick controller;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    
    camera1 = CameraServer.startAutomaticCapture(0);
    camera2 = CameraServer.startAutomaticCapture(1);

    camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    camera2.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    
    m_robotContainer.getDriveTrainSubsystem();
    DrivetrainSubsystem.zeroHeading();
    m_robotContainer.getDriveTrainSubsystem().resetEncoders();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    
    m_robotContainer.getDriveTrainSubsystem();
    DrivetrainSubsystem.zeroHeading();
    m_robotContainer.getDriveTrainSubsystem().resetEncoders();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    stabilize.initialize();

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // System.out.println(DrivetrainSubsystem.navX.getPitch());
    // System.out.println(DrivetrainSubsystem.navX.getYaw());
    // System.out.println(DrivetrainSubsystem.navX.getRoll());

    // SmartDashboard.putNumber("Gryoscope Pitch Value", DrivetrainSubsystem.navX.getPitch());
    // SmartDashboard.putNumber("Gryoscope Yaw Value", DrivetrainSubsystem.navX.getYaw());
    // SmartDashboard.putNumber("Gryoscope Roll Value", DrivetrainSubsystem.navX.getRoll());

    if (RobotContainer.joystick.getRawButtonPressed(2)){
      System.out.println(2.0);
      stabilize.execute();

    }
    
    if (RobotContainer.joystick.getRawButtonPressed(4));{
      System.out.println(4.0);
      pneumatic.getPotentiometerAngle();
    }
    
    if (RobotContainer.joyStick.getRawButtonPressed(1)) {
      if (cameraNum == 1){
      System.out.println("Setting camera 2");
      cameraSelection.setString(camera2.getName());
      cameraNum = 2;
      }
      else if (cameraNum == 2){
        System.out.println("Setting camera 1");
        cameraSelection.setString(camera1.getName());
        cameraNum = 1;
      }
    } 
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
