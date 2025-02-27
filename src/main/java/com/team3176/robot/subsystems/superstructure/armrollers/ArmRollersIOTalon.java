// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3176.robot.subsystems.superstructure.armrollers;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import com.team3176.robot.constants.Hardwaremap;
import com.team3176.robot.constants.SuperStructureConstants;
import com.team3176.robot.util.TalonUtils;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;

/** Template hardware interface for a closed loop subsystem. */
public class ArmRollersIOTalon implements ArmRollersIO {

  private TalonFX rollerController;
  VelocityVoltage voltVelocity;
  VoltageOut rollerVolts = new VoltageOut(0.0);
  
  DigitalInput rollerLinebreak;
  DigitalInput pivotLinebreak;

  private final StatusSignal<Voltage> rollerAppliedVolts;
  private final StatusSignal<Current> rollerCurrentAmpsStator;
  private final StatusSignal<Current> rollerCurrentAmpsSupply;
  private final StatusSignal<AngularVelocity> rollerVelocity;
  private final StatusSignal<Temperature> rollerTemp;
  private LaserCan lc;
  private LaserCan.Measurement measurement;
  public boolean hasCoral = false;
  private double coralDistance = 100; 
  

  public ArmRollersIOTalon() {

    TalonFXConfiguration rollerConfigs = new TalonFXConfiguration();
    lc = new LaserCan(Hardwaremap.laserCan_CID);
    // Optionally initialise the settings of the LaserCAN, if you haven't already done so in GrappleHook
    try {
      lc.setRangingMode(LaserCan.RangingMode.SHORT);
      lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
    // voltVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    // voltPosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

    // rollerLinebreak = new DigitalInput(Hardwaremap.armRollerLinebreak_DIO);
    // pivotLinebreak = new DigitalInput(Hardwaremap.armPivotLinebreak_DIO);
    rollerController = new TalonFX(Hardwaremap.armRoller_CID, Hardwaremap.armRoller_CBN);


    TalonUtils.applyTalonFxConfigs(rollerController, rollerConfigs);


    rollerAppliedVolts = rollerController.getMotorVoltage();
    rollerCurrentAmpsStator = rollerController.getStatorCurrent();
    rollerCurrentAmpsSupply = rollerController.getSupplyCurrent();
    rollerVelocity = rollerController.getVelocity();
    rollerTemp = rollerController.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        rollerAppliedVolts,
        rollerVelocity,
        rollerCurrentAmpsStator,
        rollerTemp,
        rollerCurrentAmpsSupply);

    rollerController.optimizeBusUtilization();
  }



  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ArmRollersIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        rollerAppliedVolts,
        rollerVelocity,
        rollerCurrentAmpsStator,
        rollerTemp,
        rollerCurrentAmpsSupply);

    // inputs.isRollerLinebreak = (!rollerLinebreak.get());
    // inputs.isPivotLinebreak = (!pivotLinebreak.get());


    inputs.rollerAppliedVolts = rollerAppliedVolts.getValueAsDouble();
    inputs.rollerAmpsStator = rollerCurrentAmpsStator.getValueAsDouble();
    inputs.rollerAmpsSupply = rollerCurrentAmpsSupply.getValueAsDouble();
    inputs.rollerTempCelcius = rollerTemp.getValueAsDouble();
    inputs.rollerVelocityRadPerSec = Units.rotationsToRadians(rollerVelocity.getValueAsDouble());    
   
    inputs.coralLaserCan = this.hasCoral;
    inputs.coralLaserCanDist = coralDistance;
  }

  @Override
  public void updateLaserCanMeasurement() {
     this.measurement = lc.getMeasurement();
     this.coralDistance = this.measurement.distance_mm;
    
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      if (this.measurement.distance_mm <= SuperStructureConstants.CORAL_DISTANCE) {
        hasCoral = true;
      } else {
        hasCoral = false;
      
      }
    /* 
    } else {
      System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");
      // You can still use distance_mm in here, if you're ok tolerating a clamped value or an unreliable measurement.
    */
    }
    
  }

  @Override
  public void setRollerVolts(double volts) {
    rollerController.setControl(rollerVolts.withOutput(volts));
  }

}
