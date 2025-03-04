
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3176.robot.subsystems.superstructure.arm;

import org.littletonrobotics.junction.AutoLog;

/** Template hardware interface for a closed loop subsystem. */
public interface ArmIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ArmIOInputs {
    public double pivotPositionDeg = 0.0;
    public double pivotPositionRot = 0.0;
    public double pivotPositionRotREAL =  0.0;
    public double pivotAbsolutePositionDegrees = 0.0;
    public double pivotVelocityRadPerSec = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double pivotAmpsStator = 0.0;
    public double pivotAmpsSupply = 0.0;
    public double pivotTempCelcius = 0.0;
    public double pivot_pos_offset = 0.0;

    public boolean coralLaserCan = false;
    public double coralLaserCanDist = 100;
    public boolean hasCoral = false;
    public boolean ispivotlinebreak = false;
    public boolean upperlimitswitch = false;
    public boolean lowerlimitswitch = false;

    // constructor if needed for some inputs
    ArmIOInputs() {}
  }

  /*   public default Boolean getRollerLinebreak() {} */

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  public default void updateLaserCanMeasurement() {}

  public default void setPivotVolts(double volts) {}

  public default void setPivotPIDPosition(double position) {}

  public default void setCoastMode(boolean isCoastMode) {}

  public default void setPivotVoltagePos(double position) {}

  public default void setPivotBrakeMode(boolean enable) {};

}
