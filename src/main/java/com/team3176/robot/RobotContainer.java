// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package com.team3176.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import com.team3176.robot.commands.DriveCommands;
import com.team3176.robot.constants.BaseConstants;

import com.team3176.robot.generated.TunerConstants;
import com.team3176.robot.subsystems.controller.Controller;
import com.team3176.robot.subsystems.drivetrain.Drivetrain;
import com.team3176.robot.subsystems.drivetrain.GyroIO;
import com.team3176.robot.subsystems.drivetrain.GyroIOPigeon2;
import com.team3176.robot.subsystems.superstructure.Superstructure;
import com.team3176.robot.subsystems.drivetrain.SwervepodIO;
import com.team3176.robot.subsystems.drivetrain.SwervepodIOSim;
import com.team3176.robot.subsystems.drivetrain.SwervepodIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drivetrain drive;
  private final Superstructure superstructure;

  

  // Controller
  private final Controller controller = Controller.getInstance();

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (BaseConstants.getMode()) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drivetrain(
                new GyroIOPigeon2(),
                new SwervepodIOTalonFX(TunerConstants.FrontLeft),
                new SwervepodIOTalonFX(TunerConstants.FrontRight),
                new SwervepodIOTalonFX(TunerConstants.BackLeft),
                new SwervepodIOTalonFX(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drivetrain(
                new GyroIO() {},
                new SwervepodIOSim(TunerConstants.FrontLeft),
                new SwervepodIOSim(TunerConstants.FrontRight),
                new SwervepodIOSim(TunerConstants.BackLeft),
                new SwervepodIOSim(TunerConstants.BackRight));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drivetrain(
                new GyroIO() {},
                new SwervepodIO() {},
                new SwervepodIO() {},
                new SwervepodIO() {},
                new SwervepodIO() {});
        break;
    }

    superstructure = Superstructure.getInstance();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drive.setDefaultCommand(
            // Drivetrain will execute this command periodically
            //drive.applyRequest(() ->
            DriveCommands.joystickDrive(
                drive, 
                () -> (controller.getForward()), // * MaxSpeed) // Drive forward with negative Y (forward)
                () -> (controller.getStrafe()), // * MaxSpeed) // Drive left with negative X (left)
                () -> (controller.getSpin()) // * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        //controller.operator.a().whileTrue(drivetrain.applyRequest(() -> brake));
        //controller.operator.b().whileTrue(drivetrain.applyRequest(() ->
        //    point.withModuleDirection(new Rotation2d(controller.operator.getLeftY(), controller.operator.getLeftX()))
        //));

        //controller.operator.pov(0).whileTrue(drivetrain.applyRequest(() ->
        //    forwardStraight.withVelocityX(0.5).withVelocityY(0))
        //);
        //controller.operator.pov(180).whileTrue(drivetrain.applyRequest(() ->
        //    forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        //);

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //controller.operator.back().and(controller.operator.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //controller.operator.back().and(controller.operator.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //controller.operator.start().and(controller.operator.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //controller.operator.start().and(controller.operator.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        //controller.rotStick.button(8).onTrue(drive.runOnce(() -> drive.seedFieldCentric()));
       
        // Reset gyro to 0° when B button is pressed
        controller.rotStick.button(8).onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));


    // Climb buttons
    controller.operator.leftBumper().whileTrue(superstructure.testClimbManual(() -> controller.operator.getLeftY()));
    
    // Scoring Positions
    controller.operator.a().onTrue(superstructure.goToL1()).onFalse(superstructure.goToL0());    
    controller.operator.x().onTrue(superstructure.goToL2()).onFalse(superstructure.goToL0());    
    controller.operator.b().onTrue(superstructure.goToL3()).onFalse(superstructure.goToL0());    
    controller.operator.b().onTrue(superstructure.goToL4()).onFalse(superstructure.goToL0());   
    
    // Human Load Positions and Rollers
    controller.operator.rightBumper().onTrue(superstructure.goToHumanLoad()).onFalse(superstructure.goToL0());

    // Shoot
    controller.transStick.button(0).onTrue(superstructure.shoot()).onFalse(superstructure.stopRollers());
    
    /* 
    controller.operator.a().onTrue(superstructure.armVoltPos()).onFalse(superstructure.arm2Home());
    controller.operator.rightTrigger(.90).whileTrue(superstructure.armVoltPosManual(() -> controller.operator.getRightY()));
    controller.operator.leftTrigger(.90).whileTrue(superstructure.armVoltVelManual(() -> controller.operator.getLeftY()));
    controller.operator.b().whileTrue(superstructure.armVoltVel());
    controller.operator.x().onTrue(superstructure.testElevator()).onFalse(superstructure.goToL0());
    controller.operator.rightBumper().whileTrue(superstructure.testElevatorManual(() -> controller.operator.getRightY()));
    */











    /* 
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    */
  }


  public void setPivotCoast() {
    superstructure.setPivotCoast();
  }

  public void setPivotBrake() {
    superstructure.setPivotBrake();
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
