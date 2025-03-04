
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3176.robot.subsystems.superstructure.armrollers;

import org.littletonrobotics.junction.AutoLog;

/** Template hardware interface for a closed loop subsystem. */
public interface ArmRollersIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ArmRollersIOInputs {

    public double rollerVelocityRadPerSec = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerAmpsStator = 0.0;
    public double rollerAmpsSupply = 0.0;
    public double rollerTempCelcius = 0.0;

    public boolean coralLaserCan = false;
    public double coralLaserCanDist = 100;
    public boolean hasCoral = false;
    public boolean ispivotlinebreak = false;
    public boolean upperlimitswitch = false;
    public boolean lowerlimitswitch = false;

    // constructor if needed for some inputs
    ArmRollersIOInputs() {}
  }

  /*   public default Boolean getRollerLinebreak() {} */

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmRollersIOInputs inputs) {}

  public default void updateLaserCanMeasurement() {}

  public default void setRollerVolts(double volts) {}

  public default void setRollerPIDVelocity(double rpm) {}

  public default void setCoastMode(boolean isCoastMode) {}

  public default void setBrakeMode(boolean enable) {};

}
