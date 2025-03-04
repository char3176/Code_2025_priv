// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3176.robot.subsystems.superstructure.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.junction.Logger;
import com.team3176.robot.constants.BaseConstants;

/** Template hardware interface for a closed loop subsystem. */
public class ArmIOSim implements ArmIO {

  private SingleJointedArmSim pivotSim;
  private FlywheelSim rollerSim;
  private double appliedVolts;

  public ArmIOSim() {
    pivotSim =
        new SingleJointedArmSim(
            DCMotor.getFalcon500(1), 20, 0.5, 0.7, -1.0 * Math.PI, 3.14, true, 0.0);
  }
  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ArmIOInputs inputs) {
    pivotSim.update(BaseConstants.LOOP_PERIODIC_SECS);
    rollerSim.update(BaseConstants.LOOP_PERIODIC_SECS);
    inputs.pivotPositionDeg = Units.radiansToDegrees(pivotSim.getAngleRads()) + 90;
    inputs.pivotVelocityRadPerSec = pivotSim.getVelocityRadPerSec();
    inputs.pivotAppliedVolts = appliedVolts;
    inputs.pivotAmpsStator = pivotSim.getCurrentDrawAmps();
    inputs.pivotTempCelcius = 0.0;
    Logger.recordOutput("Arm/SimPivotPos", pivotSim.getAngleRads());
  }

  @Override
  public void setPivotVolts(double volts) {
    if (DriverStation.isEnabled()) {
      appliedVolts = volts;
    } else {
      appliedVolts = 0.0;
    }
    appliedVolts = MathUtil.clamp(appliedVolts, -12, 12);
    pivotSim.setInputVoltage(appliedVolts);
  }
}
