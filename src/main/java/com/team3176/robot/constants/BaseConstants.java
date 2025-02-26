// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team3176.robot.constants;

import edu.wpi.first.wpilibj.RobotBase;
import java.util.Map;

public final class BaseConstants {
  private static final RobotType robot = RobotType.ROBOT_2025C;
  public static final double LOOP_PERIODIC_SECS = 0.02;
  public static final boolean TUNING_MODE = true;
  public static final boolean VISION_CONNECTED = false;
  public static boolean invalidRobotAlertSent = false;

  public static RobotType getRobot() {
    if (!disableHAL && RobotBase.isReal()) {
      if (robot == RobotType.ROBOT_SIMBOT) { // Invalid robot selected
        if (!invalidRobotAlertSent) {
          invalidRobotAlertSent = true;
        }
        return RobotType.ROBOT_2025C;
      } else {
        return robot;
      }
    } else {
      return robot;
    }
  }

  public static Mode getMode() {
    switch (getRobot()) {
      case ROBOT_2025C:
        return Mode.REAL;
      case CTRL_BOARD:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

      case ROBOT_SIMBOT:
        return Mode.SIM;

      default:
        return Mode.REAL;
    }
  }

  public static final Map<RobotType, String> logFolders =
      Map.of(RobotType.ROBOT_2025C, "/media/sda1/");

  public static enum RobotType {
    ROBOT_2025C,
    ROBOT_2024C,
    ROBOT_DEFENSE,
    CTRL_BOARD,
    ROBOT_SIMBOT
  }

  public static enum Mode {
    REAL,
    REPLAY,
    SIM
  }

  public static enum Status {
    STABLE,
    OK,
    OPTIONALCHECK,
    WARNING,
    GOOD,
    ERROR,
    CONE,
    CUBE,
    CONEFLASH,
    CUBEFLASH,
    NONE
  }

  // Function to disable HAL interaction when running without native libs
  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  /** Checks whether the robot the correct robot is selected when deploying. */
  public static void main(String... args) {
    if (robot == RobotType.ROBOT_SIMBOT) {
      System.err.println("Cannot deploy, invalid robot selected: " + robot.toString());
      System.exit(1);
    }
  }
}
