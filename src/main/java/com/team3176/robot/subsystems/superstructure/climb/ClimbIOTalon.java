// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3176.robot.subsystems.superstructure.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import com.team3176.robot.constants.Hardwaremap;
import com.team3176.robot.constants.SuperStructureConstants;
import com.team3176.robot.util.TalonUtils;

/** Template hardware interface for the Elevator subsystem. */
public class ClimbIOTalon implements ClimbIO {

  TalonFX climb;
  PositionVoltage voltPosition;
  NeutralOut brake;
  TalonFXConfiguration configs;
  private final StatusSignal<Angle> Position;
  private final StatusSignal<Double> PositionError;
  private final StatusSignal<Voltage> Volts;
  private final StatusSignal<Current> Amps;

  public ClimbIOTalon() {
    configs = new TalonFXConfiguration();
    brake = new NeutralOut();
    voltPosition = new PositionVoltage(0);
    climb = new TalonFX(Hardwaremap.climb_CID, Hardwaremap.climb_CBN);
    // config setting
    configs.Slot0.kP = 2.4; // An error of 0.5 rotations results in 1.2 volts output
    configs.Slot0.kI = 0.0; // A change of 1 rotation per second results in 0.1 volts output
    configs.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
    configs.Slot0.kV = 0.0; // A change of 1 rotation per second results in 0.1 volts output
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        SuperStructureConstants.CLIMB_MAXRETRACT_POS;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        SuperStructureConstants.CLIMB_MAXDEPLOY_POS;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    TalonUtils.applyTalonFxConfigs(climb, configs);

    Position = climb.getPosition();
    PositionError = climb.getClosedLoopError();
    Amps = climb.getStatorCurrent();
    Volts = climb.getMotorVoltage();

    climb.setPosition(0.0);
    BaseStatusSignal.setUpdateFrequencyForAll(50, Position, PositionError, Amps, Volts);
    climb.optimizeBusUtilization();
  }
  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    BaseStatusSignal.refreshAll(Position, PositionError, Amps, Volts);
    inputs.Position = Position.getValueAsDouble();
    inputs.PositionError = PositionError.getValue();
    inputs.AmpsStator = Amps.getValueAsDouble();
    inputs.Volts = Volts.getValueAsDouble();
  }

  @Override
  public void setPIDPosition(double position) {
    climb.setControl(voltPosition.withPosition(position));
  }

  @Override
  public void set(double percent) {
    climb.set(percent);
  }

  @Override
  public void setVoltage(double voltage) {
    climb.setVoltage(voltage);
  }

  @Override
  public void setVoltge(double voltage) {
    climb.setVoltage(voltage);
  }

 @Override
  public void setBrakeMode(boolean enable) {
    if (enable) {
      climb.setNeutralMode(NeutralModeValue.Brake);
    } else {
      climb.setNeutralMode(NeutralModeValue.Coast);
    }
  }

}