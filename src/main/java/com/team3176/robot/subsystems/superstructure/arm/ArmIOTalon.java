// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3176.robot.subsystems.superstructure.arm;

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
public class ArmIOTalon implements ArmIO {

  private TalonFX pivotController;
  private CANcoder armPivotEncoder;
  VelocityVoltage voltVelocity;
  VoltageOut pivotVolts = new VoltageOut(0.0);
  PositionVoltage voltPosition = new PositionVoltage(0);
  private Rotation2d encoderOffset; 
  private double pivot_pos_offset = 0;
  
  DigitalInput pivotLinebreak;

  private final StatusSignal<Voltage> pivotAppliedVolts;
  private final StatusSignal<Current> pivotCurrentAmpsStator;
  private final StatusSignal<Current> pivotCurrentAmpsSupply;
  private final StatusSignal<AngularVelocity> pivotVelocity;
  private final StatusSignal<Angle> pivotPosition;
  private final StatusSignal<Angle> pivotAbsolutePosition;
  private final StatusSignal<Temperature> pivotTemp;

  private LaserCan lc;
  //private LaserCan.Measurement measurement;
  public boolean hasCoral = false;
  private double coralDistance = 100; 
  

  public ArmIOTalon() {

    TalonFXConfiguration rollerConfigs = new TalonFXConfiguration();
    TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();
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
    pivotController = new TalonFX(Hardwaremap.armPivot_CID, Hardwaremap.armPivot_CBN);

    armPivotEncoder = new CANcoder(Hardwaremap.armCancoder_CID, Hardwaremap.armPivot_CBN);
    //private Rotation2d offset; 

    // Position w/ rollers touching back of Falcon at boot = 0.22
    // Position w/ rollers nearly touching the gears on front at boot = 1.2
    var pivotEncoderConfig = new CANcoderConfiguration();
    encoderOffset = Rotation2d.fromDegrees(SuperStructureConstants.ARM_ENCODER_OFFSET);
    //pivotEncoderConfig.MagnetSensor.MagnetOffset = encoderOffset.getRotations();
    

    //armPivotEncoder.getConfigurator().apply(pivotEncoderConfig);

    pivotConfigs.Slot0.kP = 60; // An error of 1 rotation results in 2.4 V output
    pivotConfigs.Slot0.kI = 0; // No output for integrated error
    pivotConfigs.Slot0.kD = 1; // A velocity of 1 rps results in 0.1 V output

    pivotConfigs.Voltage.PeakForwardVoltage = 8;
    pivotConfigs.Voltage.PeakReverseVoltage = -10;
    pivotConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    pivotConfigs.Feedback.FeedbackRemoteSensorID = Hardwaremap.armCancoder_CID;
    pivotConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    pivotConfigs.Feedback.SensorToMechanismRatio = 1.0;

    pivotConfigs.CurrentLimits.SupplyCurrentLimit = 60;
    pivotConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        1.8;
    pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        0.2;
    pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true; 

    TalonUtils.applyTalonFxConfigs(pivotController, pivotConfigs);
    //pivotController.setPosition(0, 0);

    pivotAppliedVolts = pivotController.getMotorVoltage();
    pivotCurrentAmpsStator = pivotController.getStatorCurrent();
    pivotCurrentAmpsSupply = pivotController.getSupplyCurrent();
    pivotVelocity = pivotController.getVelocity();
    pivotPosition = armPivotEncoder.getPositionSinceBoot();
    pivotAbsolutePosition = armPivotEncoder.getAbsolutePosition();
    pivotTemp = pivotController.getDeviceTemp();

    pivot_pos_offset = armPivotEncoder.getPosition().getValueAsDouble();


    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        pivotAppliedVolts,
        pivotCurrentAmpsStator,
        pivotVelocity,
        pivotPosition,
        pivotTemp,
        pivotCurrentAmpsSupply);

    pivotController.optimizeBusUtilization();
  }



  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        pivotAppliedVolts,
        pivotCurrentAmpsStator,
        pivotVelocity,
        pivotPosition,
        pivotTemp,
        pivotCurrentAmpsSupply
        );

    // inputs.isRollerLinebreak = (!rollerLinebreak.get());
    // inputs.isPivotLinebreak = (!pivotLinebreak.get());

    inputs.pivotAppliedVolts = pivotAppliedVolts.getValueAsDouble();
    inputs.pivotAmpsStator = pivotCurrentAmpsStator.getValueAsDouble();
    inputs.pivotAmpsSupply = pivotCurrentAmpsSupply.getValueAsDouble();
    inputs.pivotTempCelcius = pivotTemp.getValueAsDouble();
    inputs.pivotPositionDeg = Units.rotationsToDegrees(pivotPosition.getValueAsDouble());
    inputs.pivot_pos_offset = pivot_pos_offset;
    inputs.pivotPositionRot = armPivotEncoder.getPosition().getValueAsDouble() - pivot_pos_offset;
    inputs.pivotPositionRotREAL = armPivotEncoder.getPosition().getValueAsDouble(); 
    inputs.pivotVelocityRadPerSec = Units.rotationsToRadians(pivotVelocity.getValueAsDouble());

    inputs.pivotAbsolutePositionDegrees =
        MathUtil.inputModulus(
            Rotation2d.fromRotations(armPivotEncoder.getAbsolutePosition().getValueAsDouble())
                .minus(encoderOffset)
                .getDegrees(),
            -180,
            180);
    inputs.coralLaserCan = this.hasCoral;
    inputs.coralLaserCanDist = coralDistance;
  }

  /*
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
   // }
    
  //}

  @Override
  public void setPivotVolts(double volts) {
    pivotController.setControl(pivotVolts.withOutput(volts));
  }

  @Override
  public void setPivotVoltagePos(double position) {
    pivotController.setControl(voltPosition.withPosition(position + pivot_pos_offset));
  }

   @Override
  public void setPivotBrakeMode(boolean enable) {
    if (enable) {
      pivotController.setNeutralMode(NeutralModeValue.Brake);
    } else {
      pivotController.setNeutralMode(NeutralModeValue.Coast);
    }
  }
}
