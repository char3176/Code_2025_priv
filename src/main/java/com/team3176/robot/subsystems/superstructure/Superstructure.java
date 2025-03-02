package com.team3176.robot.subsystems.superstructure;

import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
//import com.team3176.robot.constants.FieldConstants;
// import java.util.function.IntSupplier;
import com.team3176.robot.subsystems.drivetrain.Drivetrain;
import com.team3176.robot.subsystems.superstructure.climb.Climb;
import com.team3176.robot.subsystems.superstructure.arm.Arm;
import com.team3176.robot.subsystems.superstructure.armrollers.ArmRollers.POS;
import com.team3176.robot.subsystems.superstructure.armrollers.ArmRollers;
import com.team3176.robot.subsystems.superstructure.elevator.Elevator;
import com.team3176.robot.util.LoggedTunableNumber;
import com.ctre.phoenix6.StatusSignal;
import com.team3176.robot.constants.SuperStructureConstants;
import com.team3176.robot.util.LoggedTunableNumber;
import com.team3176.robot.util.TunablePID;
public class Superstructure {
  private static Superstructure instance;
  private Climb climb;
  private Arm arm;
  private ArmRollers armrollers;
  private Elevator elevator;
  private final LoggedTunableNumber pivotTuneSetPoint, velTuneSetPoint, elevTunePositionSetPoint, climbTunePositionSetPoint;
  private final LoggedTunableNumber L1ElvSetpoint, L2ElvSetpoint, L3ElvSetpoint, L4ElvSetpoint;
  private final LoggedTunableNumber HumanLoadElvSetpoint;
  private final LoggedTunableNumber HumanLoadTuneSetpoint, L0TuneSetpoint, L1TuneSetpoint, L2TuneSetpoint, L3TuneSetpoint, L4TuneSetpoint;
  private final LoggedTunableNumber HumanLoadTuneVolts, L0TuneShootingVolts, L1TuneShootingVolts, L2TuneShootingVolts, L3TuneShootingVolts, L4TuneShootingVolts;

  public Superstructure() {
    climb = Climb.getInstance();
    arm = Arm.getInstance();
    armrollers = ArmRollers.getInstance();
    elevator = Elevator.getInstance();
    this.pivotTuneSetPoint = new LoggedTunableNumber("ss/pivotSetpoint", 0);
    this.velTuneSetPoint = new LoggedTunableNumber("ss/velSetpoint", 0);
    this.elevTunePositionSetPoint = new LoggedTunableNumber("ss/posSetpoint", 0);
    this.climbTunePositionSetPoint = new LoggedTunableNumber("ss/posSetpoint", 0);
    this.HumanLoadElvSetpoint = new LoggedTunableNumber("ss/ElvL1setpoint", 0);
    this.L1ElvSetpoint = new LoggedTunableNumber("ss/ElvL1setpoin", SuperStructureConstants.ELEVATORLEADER_L1_POS);
    this.L2ElvSetpoint = new LoggedTunableNumber("ss/ElvL2setpoin", SuperStructureConstants.ELEVATORLEADER_L2_POS);
    this.L3ElvSetpoint = new LoggedTunableNumber("ss/ElvL3setpoin", SuperStructureConstants.ELEVATORLEADER_L3_POS);
    this.L4ElvSetpoint = new LoggedTunableNumber("ss/ElvL4setpoint", SuperStructureConstants.ELEVATORLEADER_L4_POS);
   this.HumanLoadTuneSetpoint = new LoggedTunableNumber("ss/HumanLoadSetpoint", SuperStructureConstants.ARM_HF_POS);
    this.L0TuneSetpoint = new LoggedTunableNumber("ss/pivL0Setpoint", SuperStructureConstants.ARM_L0_POS);
    this.L1TuneSetpoint = new LoggedTunableNumber("ss/pivL1Setpoint", SuperStructureConstants.ARM_L1_POS);
    this.L2TuneSetpoint = new LoggedTunableNumber("ss/pivL2Setpoint", SuperStructureConstants.ARM_L2_POS);
    this.L3TuneSetpoint = new LoggedTunableNumber("ss/pivL3Setpoint", SuperStructureConstants.ARM_L3_POS);
    this.L4TuneSetpoint = new LoggedTunableNumber("ss/pivL4Setpoint", SuperStructureConstants.ARM_L4_POS);
    this.HumanLoadTuneVolts = new LoggedTunableNumber("ss/HumanLoadVolts", -SuperStructureConstants.ARM_L3_SHOOTINGVOLTS);
    this.L0TuneShootingVolts = new LoggedTunableNumber("ss/L0Volts", SuperStructureConstants.ARM_L0_SHOOTINGVOLTS);
    this.L1TuneShootingVolts = new LoggedTunableNumber("ss/L1Volts", SuperStructureConstants.ARM_L1_SHOOTINGVOLTS);
    this.L2TuneShootingVolts = new LoggedTunableNumber("ss/L2Volts", SuperStructureConstants.ARM_L2_SHOOTINGVOLTS);
    this.L3TuneShootingVolts = new LoggedTunableNumber("ss/L3Volts", SuperStructureConstants.ARM_L3_SHOOTINGVOLTS);
    this.L4TuneShootingVolts = new LoggedTunableNumber("ss/L4Volts", SuperStructureConstants.ARM_L4_SHOOTINGVOLTS);
  }

  public Command armVoltPos() {
    return arm.runPosition(()->this.pivotTuneSetPoint.get());
  }

  public Command arm2Home() {
    return arm.arm2Home();
  }

  public Command setPivotCoast() {
    return arm.setPivot2Coast();
  }

  public Command setPivotBrake() {
    return arm.setPivot2Brake();
  }

  public Command setClimbCoast() {
    return climb.set2Coast();
  }

  public Command setClimbBrake() {
    return climb.set2Brake();
  }

  public Command armVoltPosManual(DoubleSupplier voltage) {
    return arm.runPosition(()->this.pivotTuneSetPoint.get());
  }

  public Command armVoltVel() {
    return (armrollers.runVelocity(()-> this.velTuneSetPoint.get())).andThen(armrollers.stopRollers());
  }
public Command armVoltVelManual(DoubleSupplier voltage) { return armrollers.runVelocity(() -> voltage.getAsDouble()); }

  public Command armRevVoltVel() {
    return armrollers.runVelocity(()->-1 * this.velTuneSetPoint.get());
  }

  public Command testElevator() {
    return elevator.goToPosition(()->this.elevTunePositionSetPoint.get());
  }
  
  public Command testElevatorManual(DoubleSupplier voltage) {
    return elevator.goToPositionManual(() -> voltage.getAsDouble());
  }

  public Command goToL0() {
    return (elevator.goToPosition(() -> SuperStructureConstants.ELEVATORLEADER_L0_POS)).alongWith(arm.runPosition(() -> SuperStructureConstants.ARM_L0_POS).andThen(armrollers.setPosTrack(POS.L0)));
  }

  public Command goToL1() {
    return (elevator.goToPosition(() -> SuperStructureConstants.ELEVATORLEADER_L1_POS).alongWith(arm.runPosition(() -> SuperStructureConstants.ARM_L1_POS).andThen(armrollers.setPosTrack(POS.L1))));
  }

  public Command goToL2() {
    return (elevator.goToPosition(() -> SuperStructureConstants.ELEVATORLEADER_L2_POS)).alongWith(arm.runPosition(() -> SuperStructureConstants.ARM_L2_POS).andThen(armrollers.setPosTrack(POS.L2)));
  }

  public Command goToL3() {
    return (elevator.goToPosition(() -> SuperStructureConstants.ELEVATORLEADER_L3_POS)).alongWith(arm.runPosition(() -> SuperStructureConstants.ARM_L3_POS).andThen(armrollers.setPosTrack(POS.L3)));
  }

  public Command goToL4() {
    return (elevator.goToPosition(() -> SuperStructureConstants.ELEVATORLEADER_L4_POS).alongWith(arm.runPosition(() -> SuperStructureConstants.ARM_L4_POS).andThen(armrollers.setPosTrack(POS.L4))));
  }

  public Command goToHumanLoad() {
    //return (elevator.goToPosition(() -> SuperStructureConstants.ELEVATORLEADER_HF_POS).alongWith(arm.runPosition(() -> SuperStructureConstants.ARM_HF_POS).andThen(arm.setPosTrack(POS.HF))));
    return (elevator.goToPosition(() -> SuperStructureConstants.ELEVATORLEADER_HF_POS).alongWith(arm.runPosition(() -> SuperStructureConstants.ARM_HF_POS).andThen(armrollers.setPosTrack(POS.HF)))); }
  public Command runRollersIn () {
    return armrollers.runVelocity(() -> SuperStructureConstants.ARM_HF_VOLTS);
    //return armrollers.runVelocity(() -> this.HumanLoadTuneVolts.get());
    //return (arm.runRollersIn(() -> this.HumanLoadTuneVolts.get()));//.until(() -> armrollers.haveCoral());
  }


  public Command shoot() {
    return (armrollers.shoot());
  }

  public Command stopRollers() {
    return (armrollers.stopRollers());
  }

  public Command testClimb() {
    return climb.moveClimbPosition(() -> this.climbTunePositionSetPoint.get());
  }

  public Command testClimbManual(DoubleSupplier climbPosition) {
    return climb.moveClimbPosition(() -> climbPosition.getAsDouble());
    //return climb.moveClimbPosition(() -> this.climbTunePositionSetPoint.get());
  }


  public Command transStickClimbExtend() {
    return climb.moveClimbPosition(() -> 1);
    //return climb.moveClimbPosition(() -> this.climbTunePositionSetPoint.get());
  }

  public Command transStickClimbRetract() {
    return climb.moveClimbPosition(() -> -1);
    //return climb.moveClimbPosition(() -> this.climbTunePositionSetPoint.get());
  }


  /* 
  public Command getProcessorCoralLeftAuto() {
    return Drivetrain.getInstance()
        .goToPoint(FieldConstants.CoralStation.leftCenterFace)
        // .andThen(Drivetrain.getInstance().chaseNote().raceWith(intakeNote()));
        .andThen(Drivetrain.getInstance().chaseNote());
  }
  */

  public static Superstructure getInstance() {
    if (instance == null) {
      instance = new Superstructure();
      System.out.println("Superstructure instance created.");
    }
    return instance;
  }


}
