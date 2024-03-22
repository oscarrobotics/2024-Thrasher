// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import java.io.File;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends LoggedRobot {
  
  private RobotContainer m_robotContainer;

  private Command m_autonomousCommand;

  private static final String LOG_DIRECTORY = "/home/lvuser/logs";

  // private static final String kDefaultAuto = "Default";
  // private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  // private final SendableChooser<String> m_chooser = new SendableChooser<>();
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  private LoggedDashboardChooser<Command> m_autoChooser =
    new LoggedDashboardChooser<>("Auto Chooser");

  @Override
  public void robotInit() {
    SetupLog();
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter(LOG_DIRECTORY));
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }
    Logger.start(); //Start logging w/AdvantageScope

    m_robotContainer = new RobotContainer();

    DataLogManager.start();
    URCL.start();

    DriverStation.startDataLog(DataLogManager.getLog());
    Pathfinding.setPathfinder(new LocalADStarAK());

    // m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    // m_chooser.addOption("My Auto", kCustomAuto);
    // SmartDashboard.putData("Auto choices", m_chooser);

    m_autoChooser.addDefaultOption("None", Commands.none());
    // m_autoChooser.addOption("Blue1", m_robotContainer.getAutoCommand1BLUE());
    // m_autoChooser.addOption("Blue2", m_robotContainer.getAutoCommand2BLUE());
    // m_autoChooser.addOption("Blue3", m_robotContainer.getAutoCommand3BLUE());
    // m_autoChooser.addOption("Red1", m_robotContainer.getAutoCommand1RED());
    // m_autoChooser.addOption("Red2", m_robotContainer.getAutoCommand2RED());
    // m_autoChooser.addOption("Red3", m_robotContainer.getAutoCommand3RED());
    // m_autoChooser.addOption("Dummy Auto", new PathPlannerAuto("New Path"));
  }

  void SetupLog(){
    // Check if the log directory exists
    var directory = new File(LOG_DIRECTORY);
    if (!directory.exists()) {
      directory.mkdir(); }
    }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutoCommand1BLUE();
    // m_autonomousCommand = m_autoChooser.get();
        // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit(
  ) {
    m_robotContainer.teleopInit();
  } 
  

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
