// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3176.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import com.team3176.robot.constants.Hardwaremap;
import com.team3176.robot.constants.SuperStructureConstants;
import com.team3176.robot.subsystems.superstructure.elevator.ElevatorIO.ElevatorIOInputs;
import com.team3176.robot.util.TalonUtils;

/** Template hardware interface for the Elevator subsystem. */
public class ElevatorIOTalon implements ElevatorIO {

  TalonFX elevatorLeftLeader, elevatorRightFollower;
  PositionVoltage voltPosition;
  NeutralOut brake;
  DigitalInput elevatortoplimitswitch, elevatorbotLimitswitch;
  TalonFXConfiguration configsLeft, configsRight;
  private final StatusSignal<Angle> leftPosition, rightPosition;
  private final StatusSignal<Double> leftError, rightError;
  private final StatusSignal<Voltage> leftVolts, rightVolts;
  private final StatusSignal<Current> leftAmps, rightAmps;

  private final PositionVoltage m_PositionTorque = new PositionVoltage(0).withSlot(0);

  public ElevatorIOTalon() {
    configsLeft = new TalonFXConfiguration();
    brake = new NeutralOut();
    voltPosition = new PositionVoltage(0);
    elevatortoplimitswitch = new DigitalInput(Hardwaremap.elevatorTopLimitSwitch_DIO);
    elevatorbotLimitswitch = new DigitalInput(Hardwaremap.elevatorBotLimitSwitch_DIO);
    elevatorLeftLeader = new TalonFX(Hardwaremap.elevatorLeft_CID, Hardwaremap.elevatorLeft_CBN);
    elevatorRightFollower =
        new TalonFX(Hardwaremap.elevatorRight_CID, Hardwaremap.elevatorRight_CBN);
    elevatorLeftLeader.getConfigurator().apply(configsLeft);
    elevatorRightFollower.getConfigurator().apply(configsLeft);
    elevatorRightFollower.setControl(new Follower(elevatorLeftLeader.getDeviceID(), true));
    elevatorLeftLeader.setSafetyEnabled(false);
    // config setting
    configsLeft.Slot0.kP = 1; // An error of 0.5 rotations results in 1.2 volts output
    configsLeft.Slot0.kI = 0.0; // A change of 1 rotation per second results in 0.1 volts output
    configsLeft.Slot0.kV = 0.0; // A change of 1 rotation per second results in 0.1 volts output
    configsLeft.TorqueCurrent.withPeakForwardTorqueCurrent(120).withPeakReverseTorqueCurrent(-120);

    configsLeft.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;

    //TODO set max height
    configsLeft.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        SuperStructureConstants.ELEVATORLEADER_TOP_POS;
    configsLeft.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    configsLeft.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        SuperStructureConstants.ELEVATORLEADER_ZERO_POS;
    configsLeft.MotorOutput.NeutralMode = NeutralModeValue.Brake;


    TalonUtils.applyTalonFxConfigs(elevatorLeftLeader, configsLeft);

    leftPosition = elevatorLeftLeader.getPosition();
    rightPosition = elevatorRightFollower.getPosition();

    leftError = elevatorLeftLeader.getClosedLoopError();
    rightError = elevatorRightFollower.getClosedLoopError();

    leftAmps = elevatorLeftLeader.getStatorCurrent();
    rightAmps = elevatorRightFollower.getStatorCurrent();

    leftVolts = elevatorLeftLeader.getMotorVoltage();
    rightVolts = elevatorRightFollower.getMotorVoltage();

    elevatorRightFollower.setPosition(0);
    elevatorLeftLeader.setPosition(0.0);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        leftPosition,
        rightPosition,
        leftError,
        rightError,
        leftAmps,
        rightAmps,
        leftVolts,
        rightVolts);
    elevatorLeftLeader.optimizeBusUtilization();
    elevatorRightFollower.optimizeBusUtilization();
  }
  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leftPosition,
        rightPosition,
        leftError,
        rightError,
        leftAmps,
        rightAmps,
        leftVolts,
        rightVolts);
    inputs.istopLimitswitch = (!elevatortoplimitswitch.get());
    inputs.isbotLimitswitch = (!elevatorbotLimitswitch.get());
    inputs.leftPosition = leftPosition.getValueAsDouble();
    inputs.leftError = leftError.getValue();
    inputs.leftAmpsStator = leftAmps.getValueAsDouble();
    inputs.leftVolts = leftVolts.getValueAsDouble();
  }

  @Override
  public void setLeftPIDPosition(double position) {
    elevatorLeftLeader.setControl(voltPosition.withPosition(position));
  }

  @Override
  public void setLeftPositionTorque(double position) {
    elevatorLeftLeader.setControl(m_PositionTorque.withFeedForward(position));
    System.out.println("setleftpt Working");
  }

  @Override
  public void setLeft(double percent) {
    elevatorLeftLeader.set(percent);
  }

  @Override
  public void setLeftVoltage(double voltage) {
    if (voltage > 0 && elevatortoplimitswitch.get()){
      elevatorLeftLeader.setVoltage(voltage);
    }
    else if(voltage < 0 && elevatorbotLimitswitch.get()){
      elevatorLeftLeader.setVoltage(voltage);
    }
    else{

      elevatorLeftLeader.setVoltage(0);
    }
  }

  @Override
  public void setElevatorVoltge(double voltage) {
    elevatorLeftLeader.setVoltage(voltage);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    if (enable) {
      elevatorLeftLeader.setNeutralMode(NeutralModeValue.Brake);
      elevatorRightFollower.setNeutralMode(NeutralModeValue.Brake);
    } else {
      elevatorLeftLeader.setNeutralMode(NeutralModeValue.Coast);
      elevatorRightFollower.setNeutralMode(NeutralModeValue.Coast);
    }
  }

}
