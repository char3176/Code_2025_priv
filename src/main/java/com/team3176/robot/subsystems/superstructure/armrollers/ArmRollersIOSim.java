// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3176.robot.subsystems.superstructure.armrollers;

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
public class ArmRollersIOSim implements ArmRollersIO {

  private FlywheelSim rollerSim;
  private double appliedVolts;

  public ArmRollersIOSim() {
    rollerSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500(1), 0.025, 1.0),
            DCMotor.getFalcon500(1),
            0.0);
  }
  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ArmRollersIOInputs inputs) {
    rollerSim.update(BaseConstants.LOOP_PERIODIC_SECS);
    inputs.rollerVelocityRadPerSec = rollerSim.getAngularVelocityRadPerSec();
    inputs.rollerAppliedVolts = appliedVolts;
    inputs.rollerAmpsStator = rollerSim.getCurrentDrawAmps();
    inputs.rollerTempCelcius = 0.0;
  }

}
