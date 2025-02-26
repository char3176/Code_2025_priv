// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3176.robot.subsystems.superstructure.climb;

import org.littletonrobotics.junction.AutoLog;

/** Template hardware interface for the Elevator subsystem. */
public interface ClimbIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ClimbIOInputs {
    public double Position = 0.0;
    public double PositionError = 0.0;
    public double Volts = 0.0;
    public double AmpsStator = 0.0;
    public boolean isLeftLimitswitch = true;

    // constructor if needed for some inputs
    ClimbIOInputs() {}
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimbIOInputs inputs) {}

  public default void set(double percentOutput) {}

  public default void setPIDPosition(double rotations) {}

  public default void setClimbVoltage(double voltage) {}

  public default void setVoltage(double voltage) {}

  public default void setVoltge(double voltage) {}

  public default void setBrakeMode(boolean enable) {}

  public default void reset() {}
}
