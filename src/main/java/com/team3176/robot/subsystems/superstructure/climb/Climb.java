package com.team3176.robot.subsystems.superstructure.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import com.team3176.robot.constants.BaseConstants;
import com.team3176.robot.constants.BaseConstants.Mode;
import com.team3176.robot.constants.BaseConstants.RobotType;
import com.team3176.robot.constants.*;
import com.team3176.robot.util.LoggedTunableNumber;
// import com.team3176.robot.subsystems.superstructure.ClimbIOInputsAutoLogged;
import com.team3176.robot.util.TunablePID;

/** Elevator handles the height of the intake from the ground. */
public class Climb extends SubsystemBase {
  private static Climb instance;
  private final ClimbIO io;
  private double leftSetPoint = 0;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
  private TunablePID pid = new TunablePID("climbLeft", 0.001, 0, 0);
  private TunablePID leftPIDController = new TunablePID("climbLeft", 1, 0, 0);
  private LoggedTunableNumber LeftClimbHeight = new LoggedTunableNumber("climbLeftHeight", 0);

  private Climb(ClimbIO io) {
    this.io = io;
    leftPIDController.setTolerance(1.0);
  }

  public Command stopClimb() {
    return this.runOnce(
      () -> { 
        setVoltage(0.0);
      });
  }

  private void setVoltage(double voltage) {
    io.setVoltage(0.0);
  }

  private void climbGoToPosition(double position) {
    if (position > SuperStructureConstants.CLIMB_MAXDEPLOY_POS) {
      position = SuperStructureConstants.CLIMB_MAXDEPLOY_POS;
    }
    if (position < 0.0) {
      position = 0.0;
    }
    io.setPIDPosition(position);
  }

  public Command setClimbPosition(DoubleSupplier position) {
    return this.runEnd(
        () -> {
          climbGoToPosition((position.getAsDouble()));
        },
        () -> io.setVoltage(0.0));
  }

//This is the command Faith was call to move Climb
  public Command moveClimbPosition(DoubleSupplier delta) {
    return this.runEnd(
        () -> {
          io.setVoltage((12 * -delta.getAsDouble()));
        },
        () -> io.setVoltage(0.0));
  }

  /** Given a double supplier run the PID until we reach the setpoint then end */
  public Command goToPosition(DoubleSupplier position) {
    return this.runEnd(
            () -> {
              climbGoToPosition(position.getAsDouble());
            },
            () -> {
              io.setVoltage(0.0);
            })
        .until(() -> leftPIDController.atSetpoint());
  }

  public Command stow() {
    return goToPosition(() -> 0.0);
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
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
    pid.checkParemeterUpdate();
  }

  public static Climb getInstance() {
    if (instance == null) {
      if (BaseConstants.getMode() == Mode.REAL && BaseConstants.getRobot() != RobotType.ROBOT_DEFENSE) {
        instance = new Climb(new ClimbIOTalon() {});
        System.out.println("Climb instance created for Mode.REAL");
      } else {
        //instance = new Climb(new ClimbIOSim() {});
        System.out.println("Climb instance created for Mode.SIM");
      }
    }
    return instance;
  }
}
