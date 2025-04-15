// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.bathtubchickens.frc2025;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathShared;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import org.bathtubchickens.frc2025.Constants.RobotType;
import org.bathtubchickens.frc2025.util.CanivoreReader;
import org.bathtubchickens.frc2025.util.LoggedTracer;
import org.bathtubchickens.frc2025.util.PhoenixUtil;
import org.bathtubchickens.frc2025.util.SystemTimeValidReader;
import org.bathtubchickens.frc2025.util.VirtualSubsystem;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.rlog.RLOGServer;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private static final double loopOverrunWarningTimeout = 0.2;
  private static final double canErrorTimeThreshold = 0.5;
  private static final double canivoreErrorTimeThreshold = 0.5;
  private static final double lowBatteryVoltage = 11.8;
  private static final double lowBatteryDisabledTime = 1.5;
  private static final double lowBatteryMinCycleCount = 10;
  private static int lowBatteryCycleCount = 0;

  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private double autoStart;
  private boolean autoMessagePrinted;
  private final Timer canIntialErrorTimer = new Timer();
  private final Timer canErrorTimer = new Timer();
  private final Timer canivoreErrorTimer = new Timer();
  private final Timer disabledTimer = new Timer();
  private final CanivoreReader canivoreReader = new CanivoreReader("*");

  private final Alert canErrorAlert =
      new Alert("CAN errors detected, robot may not be controllable.", AlertType.kError);
  private final Alert canivoreErrorAlert =
      new Alert("CANivore errors detected, robot may not be controllable", AlertType.kError);
  private final Alert lowBatteryAlert =
      new Alert(
          "Battery voltage is very low, consider turning off the robot or replacing the battery.",
          AlertType.kWarning);
  private final Alert jitAlert =
      new Alert("Please wait to enable, JITing in progress.", AlertType.kError);

  public Robot() {
    Logger.recordMetadata("Robot", Constants.getRobot().toString());
    Logger.recordMetadata("Tuning Mode", Boolean.toString(Constants.tuningMode));
    Logger.recordMetadata("Field Type", FieldConstants.fieldType.toString());
    Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All Changes Committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncommitted Changes");
      case 2:
        Logger.recordMetadata("GitDirty", "Unknown");
    }

    switch (Constants.getMode()) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new RLOGServer());
        if (Constants.getRobot() == RobotType.COMPBOT) {
          LoggedPowerDistribution.getInstance(1, ModuleType.kRev);
        }
        break;

      case SIM:
        Logger.addDataReceiver(new RLOGServer());
        break;

      case REPLAY:
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    SignalLogger.enableAutoLogging(false);

    try {
      Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
      watchdogField.setAccessible(true);
      Watchdog watchdog = (Watchdog) watchdogField.get(this);
      watchdog.setTimeout(loopOverrunWarningTimeout);
    } catch (Exception e) {
      DriverStation.reportWarning("Failed to disable loop overrun warnings", false);
    }
    CommandScheduler.getInstance().setPeriod(loopOverrunWarningTimeout);

    SystemTimeValidReader.start();

    DriverStation.silenceJoystickConnectionWarning(true);

    var mathShared = MathSharedStore.getMathShared();
    MathSharedStore.setMathShared(
        new MathShared() {
          @Override
          public void reportError(String error, StackTraceElement[] stackTrace) {
            if (error.startsWith("x and y components of Rotation2d are zero")) {
              return;
            }
            mathShared.reportError(error, stackTrace);
          }

          @Override
          public void reportUsage(MathUsageId id, int count) {
            mathShared.reportUsage(id, count);
          }

          @Override
          public double getTimestamp() {
            return mathShared.getTimestamp();
          }
        });

    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String name = command.getName();
          int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
          commandCounts.put(name, count);
          Logger.recordOutput(
              "CommandsUnique" + name + "_" + Integer.toHexString(command.hashCode()), active);
          Logger.recordOutput("Commands/All" + name, count > 0);
        };
    CommandScheduler.getInstance()
        .onCommandInitialize((Command command) -> logCommandFunction.accept(command, true));
    CommandScheduler.getInstance()
        .onCommandFinish((Command command) -> logCommandFunction.accept(command, false));
    CommandScheduler.getInstance()
        .onCommandInterrupt((Command command) -> logCommandFunction.accept(command, false));

    canIntialErrorTimer.restart();
    canErrorTimer.restart();
    canivoreErrorTimer.restart();
    disabledTimer.restart();

    RobotController.setBrownoutVoltage(6.0);

    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {

    LoggedTracer.reset();
    PhoenixUtil.refreshAll();
    LoggedTracer.record("PheonixRefresh");

    VirtualSubsystem.periodicAll();

    CommandScheduler.getInstance().run();
    LoggedTracer.record("Commands");

    if (autonomousCommand != null) {
      if (!autonomousCommand.isScheduled() && !autoMessagePrinted) {
        if (DriverStation.isAutonomousEnabled()) {
          System.out.printf(
              "*** Auto finished in %.2f secs ***%n", Timer.getFPGATimestamp() - autoStart);
        } else {
          System.out.printf(
              "*** Auto cancelled in %.2f secs ***%n", Timer.getFPGATimestamp() - autoStart);
        }
        autoMessagePrinted = true;
      }
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

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
