package com.team3176.robot.subsystems.superstructure.arm;

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
import com.team3176.robot.constants.*;
import com.team3176.robot.util.LoggedTunableNumber;
import com.team3176.robot.util.TunablePID;

public class Arm extends SubsystemBase {
  private static Arm instance;
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final LoggedTunableNumber rollerVolts;
  private final LoggedTunableNumber pivotTuneSetPoint;
  private final LoggedTunableNumber HumanLoadTuneSetpoint, L0TuneSetpoint, L1TuneSetpoint, L2TuneSetpoint, L3TuneSetpoint, L4TuneSetpoint;
  private final LoggedTunableNumber HumanLoadTuneVolts, L0TuneShootingVolts, L1TuneShootingVolts, L2TuneShootingVolts, L3TuneShootingVolts, L4TuneShootingVolts; 
  private final TunablePID pivotPID;
  private Timer deployTime = new Timer();
  private double pivotSetpoint;
  private final double DEPLOY_POS = 2.1;
  private double pivot_offset = 0;
  private boolean ishomed = false;
  private double lastRollerSpeed = 0.0;
  private double pivotHome = 0.2;
  private double HumanLoadSetpoint, L0Setpoint, L1Setpoint, L2Setpoint, L3Setpoint, L4Setpoint;
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

  private enum pivotStates {
    DEPLOY,
    RETRACT,
    IDLE,
    HOLD,
  };

  private pivotStates pivotState = pivotStates.HOLD;
  // DigitalInput linebreak1 = new DigitalInput(Hardwaremap.ArmRollerLinebreak_DIO);

  private Arm(ArmIO io) {
    this.io = io;
    this.pivotPID = new TunablePID("ArmPivot", 3.0, 0.0, 0.0);
    this.rollerVolts = new LoggedTunableNumber("Arm/rollerVolts", 7.0);
    this.pivotTuneSetPoint = new LoggedTunableNumber("Arm/pivotSetpoint", 0);
    this.HumanLoadTuneSetpoint = new LoggedTunableNumber("Arm/HumanLoadSetpoint", 0.20);
    this.L0TuneSetpoint = new LoggedTunableNumber("Arm/L0Setpoint", 0.20);
    this.L1TuneSetpoint = new LoggedTunableNumber("Arm/L1Setpoint", 0.20);
    this.L2TuneSetpoint = new LoggedTunableNumber("Arm/L2Setpoint", 0.20);
    this.L3TuneSetpoint = new LoggedTunableNumber("Arm/L3Setpoint", 0.20);
    this.L4TuneSetpoint = new LoggedTunableNumber("Arm/L4Setpoint", 0.20);
    this.HumanLoadTuneVolts = new LoggedTunableNumber("Arm/HumanLoadVolts", 0.20);
    this.L0TuneShootingVolts = new LoggedTunableNumber("Arm/L0Volts", 0.20);
    this.L1TuneShootingVolts = new LoggedTunableNumber("Arm/L1Volts", 0.20);
    this.L2TuneShootingVolts = new LoggedTunableNumber("Arm/L2Volts", 0.20);
    this.L3TuneShootingVolts = new LoggedTunableNumber("Arm/L3Volts", 0.20);
    this.L4TuneShootingVolts = new LoggedTunableNumber("Arm/L4Volts", 0.20);
    this.pivotHome = inputs.pivotPositionRot;


    HumanLoadSetpoint = SuperStructureConstants.ARM_HF_POS;
    L0Setpoint = SuperStructureConstants.ARM_L0_POS;
    L1Setpoint = SuperStructureConstants.ARM_L0_POS;
    L2Setpoint = SuperStructureConstants.ARM_L2_POS;
    L3Setpoint = SuperStructureConstants.ARM_L3_POS;
    L4Setpoint = SuperStructureConstants.ARM_L4_POS;
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

  private void runPivot(double volts) {
    // this assumes positive voltage deploys the Arm and negative voltage retracts it.
    // invert the motor if that is NOT true
    io.setPivotVolts(volts);
  }

  public static Arm getInstance() {
    if (instance == null) {
      if (BaseConstants.getMode() == Mode.REAL && BaseConstants.getRobot() != RobotType.ROBOT_DEFENSE) {
        instance = new Arm(new ArmIOTalon() {});
      } else {
        instance = new Arm(new ArmIOSim() {});
      }
    }
    return instance;
  }

  public boolean haveCoral() {
    return inputs.hasCoral;
  }

  // Example command to show how to set the pivot state
  public Command reefLevelPivot(double reefLevel) {
    return this.runOnce(
        () -> {
          this.pivotSetpoint = reefLevel;
          deployTime.restart();
        });
  }

  public Command stopRollers() {
    return this.runOnce(() -> {setRollerVolts(0.0);});
  }

  public Command arm2Home() {
    return this.runOnce(
      () -> {
       setPivotVoltagePos(pivotHome); 
      }); 
    }

  // TODO: might need to deploy the Arm during a spit but maybe not

  public Command runPosition(DoubleSupplier position) {
    return this.run(
      () -> { 
        setPivotVoltagePos(position.getAsDouble());
      });
  }

  public Command runPositionVoltageManual(DoubleSupplier position) {
    return this.runEnd(
      () -> {
        setPivotVolts(position.getAsDouble());
      }, 
      () -> {
        setPivotVolts(0.0);
      });
  }

  public Command runVelocity(DoubleSupplier volts) {
    return this.runEnd(
      () -> {
        setRollerVolts(volts.getAsDouble());
      }, 
      () -> { 
        setRollerVolts(0); 
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
        setRollerVolts(L0ShootingVolts);
        break;
      case L1:
        setRollerVolts(L1ShootingVolts);
        break;
      case L2:
        setRollerVolts(L2ShootingVolts);
        break;
      case L3:
        setRollerVolts(L3ShootingVolts);
        break;
      case L4:
        setRollerVolts(L4ShootingVolts);
        break;
    }
  }

  public Command runRollersIn() {
    return this.run(
      () -> {
        setRollerVolts(HumanLoadVolts);
      });
  }



  private void setRollerVolts(double volts) {
    io.setRollerVolts(12 * volts);
  }

  private void setPivotVolts(double volts) {
    io.setPivotVolts(volts);
  }

  private void setPivotVoltagePos(double position) {
    io.setPivotVoltagePos(position);
  }

    public void setPivotCoast() {
    io.setPivotBrakeMode(false);
  }

  public void setPivotBrake() {
    io.setPivotBrakeMode(true);
  }
  
  public Command setPivot2Coast() {
    return this.runOnce(
      () -> {
        setPivotCoast();
      }); 
    }

  public Command setPivot2Brake() {
    return this.runOnce(
      () -> {
        setPivotCoast();
      }); 
    }



  @Override
  public void periodic() {
    io.updateLaserCanMeasurement();
    io.updateInputs(inputs);
    if (HumanLoadTuneSetpoint.hasChanged(hashCode())) {
      HumanLoadSetpoint = HumanLoadTuneSetpoint.get();
    }
    if (L0TuneSetpoint.hasChanged(hashCode())) {
      L0Setpoint = L0TuneSetpoint.get();
    }
    if (L1TuneSetpoint.hasChanged(hashCode())) {
      L1Setpoint = L1TuneSetpoint.get();
    }
    if (L2TuneSetpoint.hasChanged(hashCode())) {
      L2Setpoint = L2TuneSetpoint.get();
    }
    if (L3TuneSetpoint.hasChanged(hashCode())) {
      L2Setpoint = L2TuneSetpoint.get();
    }
    if (L4TuneSetpoint.hasChanged(hashCode())) {
      L4Setpoint = L4TuneSetpoint.get();
    }
    if (HumanLoadTuneVolts.hasChanged(hashCode())) {
      HumanLoadVolts = HumanLoadTuneVolts.get();
    }
    if (L0TuneSetpoint.hasChanged(hashCode())) {
      L0ShootingVolts = L0TuneShootingVolts.get();
    }
    if (L1TuneSetpoint.hasChanged(hashCode())) {
      L1ShootingVolts = L1TuneShootingVolts.get();
    }
    if (L2TuneSetpoint.hasChanged(hashCode())) {
      L2ShootingVolts = L2TuneShootingVolts.get();
    }
    if (L3TuneSetpoint.hasChanged(hashCode())) {
      L3ShootingVolts = L3TuneShootingVolts.get();
    }
    if (L4TuneSetpoint.hasChanged(hashCode())) {
      L4ShootingVolts = L4TuneShootingVolts.get();
    }






    Logger.processInputs("Arm", inputs);
    Logger.recordOutput("Arm/state", pivotState);
    if (this.pivotTuneSetPoint.hasChanged(hashCode())){


    }
    double pivot_pos = inputs.pivotPositionRot - pivot_offset;
    if (!ishomed && pivotSetpoint > 1.0) {
      pivot_pos = -3.0;
    }
    double commandVolts = pivotPID.calculate(pivot_pos, pivotSetpoint);
    if (pivot_pos <= 0.7) {
      commandVolts *= 1.6;
    }
    commandVolts = MathUtil.clamp(commandVolts, -3.5, 2.0);

    Logger.recordOutput("Arm/PID_out", commandVolts);
    Logger.recordOutput("Arm/setpoint", this.pivotSetpoint);
    Logger.recordOutput("Arm/offsetPos", pivot_pos);
    // runPivot(commandVolts);
    pivotPID.checkParemeterUpdate();
    lastRollerSpeed = inputs.rollerVelocityRadPerSec;
  }
}
