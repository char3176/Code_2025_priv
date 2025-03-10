// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3176.robot.subsystems.superstructure.elevator;

import org.littletonrobotics.junction.AutoLog;

/** Template hardware interface for the Elevator subsystem. */
public interface ElevatorIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ElevatorIOInputs {
    public double leftPosition = 0.0;
    public double rightPosition = 0.0;
    public double leftError = 0.0;
    public double rightError = 0.0;
    public double leftVolts = 0.0;
    public double rightVolts = 0.0;
    public double LeftElevatorHeight = 0;
    public double leftAmpsStator = 0.0;
    public double rightAmpsStator = 0.0;
    public double desiredrotation = 1.0;
    public boolean istopLimitswitch = true;
    public boolean isbotLimitswitch = true;
    public int periodic = 0;
    // constructor if needed for some inputs
    ElevatorIOInputs() {}
  }

  Object istopLimitswitch = null;

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setLeft(double percentOutput) {}

  public default void setLeftElevatorH(double height) {}
  // public default void setRight(double percentOutput) {}

  public default void setLeftPIDPosition(double rotations) {}

  public default void setLeftPositionTorque(double position) {}
  // public default void setRightPIDPosition(double rotations) {}

  public default void setElevatorVoltage(double voltage) {}

  public default void setBrakeMode(boolean enable) {}


  // public default void stopLeft() {}

  // public default void stopRight() {}

  // public default void setCoastMode(boolean isCoastMode) {}

  // public default void setclimbLBLimitswitchZero() {}

  // public default void setclimbRBLimitswitchZero() {}

  public default void setLeftVoltage(double voltage) {}

  // public default void setRightVoltage(double voltage) {}

  public default void setElevatorVoltge(double voltage) {}

  public default void reset() {}
}
