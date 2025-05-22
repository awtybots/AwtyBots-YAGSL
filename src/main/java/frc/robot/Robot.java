// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.management.ManagementFactory;
import java.lang.management.MemoryMXBean;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final MemoryMXBean memoryBean = ManagementFactory.getMemoryMXBean();
  private UsbCamera camera;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Initialize camera with optimized settings
    camera = CameraServer.startAutomaticCapture();
    camera.setResolution(320, 240); // Lower resolution for better performance
    camera.setFPS(15); // Lower FPS for better performance
    
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
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

    // Update performance metrics
    updatePerformanceMetrics();
    
    // Update camera telemetry with error handling
    try {
      if (camera != null) {
        SmartDashboard.putNumber("Camera/FPS", camera.getActualFPS());
        SmartDashboard.putNumber("Camera/Resolution/Width", camera.getVideoMode().width);
        SmartDashboard.putNumber("Camera/Resolution/Height", camera.getVideoMode().height);
      }
    } catch (Exception e) {
      // Only log the error if we're in test mode
      if (DriverStation.isTest()) {
        System.out.println("Camera telemetry error: " + e.getMessage());
      }
    }
  }

  private void updatePerformanceMetrics() {
    // Update CPU usage (approximate)
    double cpuUsage = ManagementFactory.getOperatingSystemMXBean().getSystemLoadAverage();
    SmartDashboard.putNumber("Robot/Performance/CPU_Usage", cpuUsage);

    // Update memory usage
    long usedMemory = memoryBean.getHeapMemoryUsage().getUsed();
    long maxMemory = memoryBean.getHeapMemoryUsage().getMax();
    double memoryUsagePercent = (double) usedMemory / maxMemory * 100;
    SmartDashboard.putNumber("Robot/Performance/Memory_Usage", memoryUsagePercent);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    // Reduce camera FPS when disabled to save resources
    camera.setFPS(5);
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.drivebase.zeroNavxGyroAuto();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    // Set camera to normal FPS for autonomous
    camera.setFPS(15);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // Set camera to normal FPS for teleop
    camera.setFPS(15);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

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
