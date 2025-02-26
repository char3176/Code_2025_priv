package com.team3176.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
//import com.team3176.robot.constants.FieldConstants;
// import java.util.function.IntSupplier;
import com.team3176.robot.subsystems.drivetrain.Drivetrain;
import com.team3176.robot.subsystems.superstructure.climb.Climb;
import com.team3176.robot.subsystems.superstructure.arm.Arm;
import com.team3176.robot.subsystems.superstructure.arm.Arm.POS;
import com.team3176.robot.subsystems.superstructure.elevator.Elevator;
import com.team3176.robot.util.LoggedTunableNumber;
import com.team3176.robot.constants.SuperStructureConstants;

public class Superstructure {
  private static Superstructure instance;
  private Climb climb;
  private Arm arm;
  private Elevator elevator;
  private final LoggedTunableNumber pivotTuneSetPoint, velTuneSetPoint, elevTunePositionSetPoint, climbTunePositionSetPoint;
  private final LoggedTunableNumber L1ElvSetpoint, L2ElvSetpoint, L3ElvSetpoint, L4ElvSetpoint;
  private final LoggedTunableNumber HumanLoadElvSetpoint;

  public Superstructure() {
    climb = Climb.getInstance();
    arm = Arm.getInstance();
    elevator = Elevator.getInstance();
    this.pivotTuneSetPoint = new LoggedTunableNumber("Arm/pivotSetpoint", 0);
    this.velTuneSetPoint = new LoggedTunableNumber("Arm/velSetpoint", 0);
    this.elevTunePositionSetPoint = new LoggedTunableNumber("Elevator/posSetpoint", 0);
    this.climbTunePositionSetPoint = new LoggedTunableNumber("climb/posSetpoint", 0);
    this.HumanLoadElvSetpoint = new LoggedTunableNumber("Elevator/L1setpoint", 0);
    this.L1ElvSetpoint = new LoggedTunableNumber("Elevator/L1setpoint", 25);
    this.L2ElvSetpoint = new LoggedTunableNumber("Elevator/L2setpoint", 50);
    this.L3ElvSetpoint = new LoggedTunableNumber("Elevator/L3setpoint", 75);
    this.L4ElvSetpoint = new LoggedTunableNumber("Elevator/L4setpoint", 107);
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
    return arm.runVelocity(()-> this.velTuneSetPoint.get());
  }

  public Command armVoltVelManual(DoubleSupplier voltage) {
    return arm.runVelocity(() -> voltage.getAsDouble());
  }

  public Command armRevVoltVel() {
    return arm.runVelocity(()->-1 * this.velTuneSetPoint.get());
  }

  public Command testElevator() {
    return elevator.goToPosition(()->this.elevTunePositionSetPoint.get());
  }
  
  public Command testElevatorManual(DoubleSupplier voltage) {
    return elevator.goToPositionManual(() -> voltage.getAsDouble());
  }

  public Command goToL0() {
    return (elevator.goToL0().alongWith(arm.runPosition(() -> SuperStructureConstants.ARM_L0_POS).alongWith(arm.setPosTrack(POS.L0))));
  }

  public Command goToL1() {
    return (elevator.goToL1().alongWith(arm.runPosition(() -> SuperStructureConstants.ARM_L1_POS).alongWith(arm.setPosTrack(POS.L1))));
  }

  public Command goToL2() {
    return (elevator.goToL1().alongWith(arm.runPosition(() -> SuperStructureConstants.ARM_L1_POS).alongWith(arm.setPosTrack(POS.L2))));
  }

  public Command goToL3() {
    return (elevator.goToL3().alongWith(arm.runPosition(() -> SuperStructureConstants.ARM_L3_POS).alongWith(arm.setPosTrack(POS.L3))));
  }

  public Command goToL4() {
    return (elevator.goToL4().alongWith(arm.runPosition(() -> SuperStructureConstants.ARM_L4_POS).alongWith(arm.setPosTrack(POS.L4))));
  }

  public Command goToHumanLoad() {
    return (elevator.goToHumanLoad().alongWith(arm.runPosition(() -> SuperStructureConstants.ARM_HF_POS).alongWith(arm.setPosTrack(POS.HF))));
  }

  public Command runRollersIn () {
    return (arm.runRollersIn()).until(() -> arm.haveCoral());
  }


  public Command shoot() {
    return (arm.shoot());
  }

  public Command stopRollers() {
    return (arm.stopRollers());
  }

  public Command testClimb() {
    return climb.moveClimbPosition(() -> this.climbTunePositionSetPoint.get());
  }

  public Command testClimbManual(DoubleSupplier climbPosition) {
    return climb.moveClimbPosition(() -> climbPosition.getAsDouble());
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
