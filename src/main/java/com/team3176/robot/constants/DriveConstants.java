// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package com.team3176.robot.constants;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

public class DriveConstants {
  public int SERIAL;
  public int THRUST_CID;
  public String THRUST_CBN;
  public int AZIMUTH_CID;
  public String AZIMUTH_CBN;
  public int CANCODER_CID;
  public String CANCODER_CBN;
  public double OFFSET;
//  public double ROBOT_MASS_KG;
//  public double ROBOT_MOI;
//  public double WHEEL_COF;
/*
 "robotWidth": 0.9,
  "robotLength": 0.9,
  "holonomicMode": true,
  "pathFolders": [],
  "autoFolders": [],
  "defaultMaxVel": 3.0,
  "defaultMaxAccel": 3.0,
  "defaultMaxAngVel": 540.0,
  "defaultMaxAngAccel": 720.0,
  "robotMass": 74.088,
  "robotMOI": 6.883,
  "robotWheelbase": 0.546,
  "robotTrackwidth": 0.546,
  "driveWheelRadius": 0.048,
  "driveGearing": 5.143,
  "maxDriveSpeed": 5.45,
  "driveMotorType": "krakenX60FOC",
  "driveCurrentLimit": 60.0,
  "wheelCOF": 1.2
*/

  public static double SWERVEPOD_AZIMUTH_REDUCTION = 12.8;
  //public static double SWERVEPOD_AZIMUTH_REDUCTION = (150.0 / 7.0);
  public static double SWERVEPOD_AZIMUTH_CURRENTLIMIT = 80;
  public static boolean SWERVEPOD_AZIMUTH_INVERTED = true;
  public static boolean SWERVEPOD_ENCODER_INVERTED = false;

  public static double SWERVEPOD_THRUST_REDUCTION = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
  public static double SWERVEPOD_THRUST_CURRENTLIMIT = 80;
  public static boolean SWERVEPOD_THRUST_INVERTED = true;

 // public static double ROBOT_MASS_KG = 71.0;
 // public static double ROBOT_MOI = .0;
 // public static double WHEEL_COF= 71;

 /* 
  // PathPlanner configuration
  public static final double robotMassKg = 74.088;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;
 public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);
}
*/


  DriveConstants() {}
}
