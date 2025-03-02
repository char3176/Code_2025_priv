package com.team3176.robot.subsystems.superstructure.armrollers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import com.team3176.robot.constants.BaseConstants;
import com.team3176.robot.constants.BaseConstants.Mode;
import com.team3176.robot.constants.BaseConstants.RobotType;
import com.team3176.robot.subsystems.superstructure.arm.Arm;
import com.team3176.robot.subsystems.superstructure.armrollers.ArmRollersIOInputsAutoLogged;
import com.team3176.robot.constants.*;
import com.team3176.robot.util.LoggedTunableNumber;
import com.team3176.robot.util.TunablePID;

public class ArmRollers extends SubsystemBase {
  private static ArmRollers instance;
  private final ArmRollersIO io;
  private final ArmRollersIOInputsAutoLogged inputs = new ArmRollersIOInputsAutoLogged();
  private final LoggedTunableNumber rollerVolts;
  private final LoggedTunableNumber HumanLoadTuneVolts, L0TuneShootingVolts, L1TuneShootingVolts, L2TuneShootingVolts, L3TuneShootingVolts, L4TuneShootingVolts; 
  private Timer deployTime = new Timer();
  private double HumanLoadVolts, L0ShootingVolts, L1ShootingVolts, L2ShootingVolts, L3ShootingVolts, L4ShootingVolts; 
  public enum POS {
    HF,
    L0,
    L1,
    L2,
    L3,
    L4,
  }
  public POS currentPosTrack = POS.L0;


  // DigitalInput linebreak1 = new DigitalInput(Hardwaremap.ArmRollerLinebreak_DIO);

  private ArmRollers(ArmRollersIO io) {
    this.io = io;
    this.rollerVolts = new LoggedTunableNumber("Arm/rollerVolts", 7.0);
    this.HumanLoadTuneVolts = new LoggedTunableNumber("Arm/HumanLoadVolts", SuperStructureConstants.ARM_HF_VOLTS);
    this.L0TuneShootingVolts = new LoggedTunableNumber("Arm/L0Volts", SuperStructureConstants.ARM_L0_SHOOTINGVOLTS);
    this.L1TuneShootingVolts = new LoggedTunableNumber("Arm/L1Volts", SuperStructureConstants.ARM_L1_SHOOTINGVOLTS);
    this.L2TuneShootingVolts = new LoggedTunableNumber("Arm/L2Volts", SuperStructureConstants.ARM_L2_SHOOTINGVOLTS);
    this.L3TuneShootingVolts = new LoggedTunableNumber("Arm/L3Volts", SuperStructureConstants.ARM_L3_SHOOTINGVOLTS);
    this.L4TuneShootingVolts = new LoggedTunableNumber("Arm/L4Volts", SuperStructureConstants.ARM_L4_SHOOTINGVOLTS);


    HumanLoadVolts = SuperStructureConstants.ARM_HF_VOLTS;
    L0ShootingVolts = SuperStructureConstants.ARM_L0_SHOOTINGVOLTS;
    L1ShootingVolts = SuperStructureConstants.ARM_L1_SHOOTINGVOLTS;
    L2ShootingVolts = SuperStructureConstants.ARM_L2_SHOOTINGVOLTS;
    L3ShootingVolts = SuperStructureConstants.ARM_L3_SHOOTINGVOLTS;
    L4ShootingVolts = SuperStructureConstants.ARM_L4_SHOOTINGVOLTS;
  }

  public Command setPosTrack(POS pos){
    return this.runOnce(() -> {
      currentPosTrack = pos;
    });
  }


  public static ArmRollers getInstance() {
    if (instance == null) {
      if (BaseConstants.getMode() == Mode.REAL && BaseConstants.getRobot() != RobotType.ROBOT_DEFENSE) {
        instance = new ArmRollers(new ArmRollersIOTalon() {});
      } else {
        instance = new ArmRollers(new ArmRollersIOSim() {});
      }
    }
    return instance;
  }

  public boolean haveCoral() {
    return inputs.hasCoral;
  }


  public Command stopRollers() {
    return this.runOnce(() -> {setRollerVolts(0.0);});
  }

  // TODO: might need to deploy the Arm during a spit but maybe not

  public Command runVelocity(DoubleSupplier volts) {
    return this.runEnd(
      () -> {
        setRollerVolts(volts.getAsDouble());
      }, 
      () -> {
        setRollerVolts(0.0);
      });
  }

  public Command shoot() {
    return this.run(
      () -> {
        runShoot();
      });
  }

  private void runShoot() {
    switch (currentPosTrack) {
      case L0:
        setRollerVolts(12);
        break;
      case L1:
        setRollerVolts(4);
        break;
      case L2:
        setRollerVolts(4);
        break;
      case L3:
        setRollerVolts(6);
        break;
      case L4:
        setRollerVolts(10);
        break;
    }
  }

  public Command runRollersIn(DoubleSupplier volts) {
    return this.runOnce(
      () -> {
        setRollerVolts(volts.getAsDouble());
      });
  }



  private void setRollerVolts(double volts) {
    io.setRollerVolts(volts);
  }

    public void setCoast() {
    io.setBrakeMode(false);
  }

  public void setBrake() {
    io.setBrakeMode(true);
  }
  
  public Command set2Coast() {
    return this.runOnce(
      () -> {
        setCoast();
      }); 
    }

  public Command set2Brake() {
    return this.runOnce(
      () -> {
        setCoast();
      }); 
    }



  @Override
  public void periodic() {
    io.updateLaserCanMeasurement();
    io.updateInputs(inputs);
    if (HumanLoadTuneVolts.hasChanged(hashCode())) {
      HumanLoadVolts = HumanLoadTuneVolts.get();
    }
    if (L0TuneShootingVolts.hasChanged(hashCode())) {
      L0ShootingVolts = L0TuneShootingVolts.get();
    }
    if (L1TuneShootingVolts.hasChanged(hashCode())) {
      L1ShootingVolts = L1TuneShootingVolts.get();
    }
    if (L2TuneShootingVolts.hasChanged(hashCode())) {
      L2ShootingVolts = L2TuneShootingVolts.get();
    }
    if (L3TuneShootingVolts.hasChanged(hashCode())) {
      L3ShootingVolts = L3TuneShootingVolts.get();
    }
    if (L4TuneShootingVolts.hasChanged(hashCode())) {
      L4ShootingVolts = L4TuneShootingVolts.get();
    }






    Logger.processInputs("Arm", inputs);



    // runPivot(commandVolts);
    //lastRollerSpeed = inputs.rollerVelocityRadPerSec;
  }
}
