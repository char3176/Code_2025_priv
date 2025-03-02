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


import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
///import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import com.team3176.robot.commands.DriveCommands;
import com.team3176.robot.constants.BaseConstants;

import com.team3176.robot.generated.TunerConstants;
import com.team3176.robot.subsystems.controller.Controller;
import com.team3176.robot.subsystems.drivetrain.Drivetrain;
import com.team3176.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import com.team3176.robot.subsystems.drivetrain.GyroIO;
import com.team3176.robot.subsystems.drivetrain.GyroIOPigeon2;
import com.team3176.robot.subsystems.superstructure.Superstructure;
import com.team3176.robot.subsystems.drivetrain.SwervepodIO;
import com.team3176.robot.subsystems.drivetrain.SwervepodIOSim;
import com.team3176.robot.subsystems.drivetrain.SwervepodIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  //private final Drivetrain drive;

  private final Superstructure superstructure;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    //private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double MaxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);



  // Controller
  private final Controller controller = Controller.getInstance();

  // Dashboard inputs
  //private final LoggedDashboardChooser<Command> autoChooser;

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
   //private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() { 

      //autoChooser = AutoBuilder.buildAutoChooser("Tests");
       // SmartDashboard.putData("Auto Mode", autoChooser);
/* 
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
   */ 

    superstructure = Superstructure.getInstance();
 
    /*
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
    */
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
        /*
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
        
        controller.transStick.button(4).onTrue(Commands.runOnce(drive::stopWithX, drive));
       */
        drivetrain.setDefaultCommand(
          // Drivetrain will execute this command periodically
          drivetrain.applyRequest(() ->
            drive.withVelocityX(controller.getForward() * MaxSpeed) // Drive forward with negative Y (forward)
              .withVelocityY(controller.getStrafe() * MaxSpeed) // Drive left with negative X (left)
              .withRotationalRate(controller.getSpin() * MaxAngularRate) // Drive counterclockwise with negative X (left)
          )
        );

        //Boost me baby (2x speed)
        controller.rotStick.button(1).whileTrue(
            drivetrain.applyRequest(() -> 
                drive.withVelocityX(controller.getForward() * MaxSpeed * 2)
                .withVelocityY(controller.getStrafe() * MaxSpeed * 2)
                .withRotationalRate(controller.getSpin() * MaxAngularRate * 2)
            )
        );

        controller.rotStick.button(8).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        controller.transStick.button(4).whileTrue(drivetrain.applyRequest(() -> brake));
        controller.rotStick.button(4).whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(controller.getForward(), controller.getStrafe()))
        ));


        controller.rotStick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        controller.rotStick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        controller.operator.back().and(controller.rotStick.button(12)).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controller.operator.back().and(controller.rotStick.button(13)).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        controller.operator.back().and(controller.rotStick.button(14)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        controller.operator.back().and(controller.rotStick.button(15)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));




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
       
        // Reset gyro to 0° 
        /* 
        controller.rotStick.button(8).onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
        */


    // Climb buttons
    // Max retraction position = -70
    // Staring configuration = 0 to -5
    // Max extension = 
    controller.operator.leftBumper().whileTrue(superstructure.testClimbManual(() -> -controller.operator.getLeftY()));
    controller.transStick.button(16).and(controller.transStick.button(15)).whileTrue(superstructure.transStickClimbExtend());
    controller.transStick.button(16).and(controller.transStick.button(14)).whileTrue(superstructure.transStickClimbRetract());
     
    // Scoring Positions
    controller.operator.a().onTrue(superstructure.goToL1()); //.onFalse(superstructure.goToL0()); 
    controller.operator.x().onTrue(superstructure.goToL2()); //.onFalse(superstructure.goToL0());    
    controller.operator.y().onTrue(superstructure.goToL3()); //.onFalse(superstructure.goToL0());    
    controller.operator.b().onTrue(superstructure.goToL4()); //.onFalse(superstructure.goToL0());   
    controller.operator.pov(180).onTrue(superstructure.goToL0()); 
    controller.transStick.button(11).onTrue(superstructure.goToL0());   
    // Human Load Positions and Rollers
    controller.operator.rightBumper().onTrue(superstructure.goToHumanLoad()); //.onFalse(superstructure.goToL0());
    controller.operator.leftTrigger(0.8).whileTrue(superstructure.runRollersIn()).onFalse(superstructure.stopRollers());

    // Shoot
    controller.transStick.button(1).onTrue(superstructure.shoot()).onFalse(superstructure.stopRollers());

    
    
      
    //controller.operator.a().onTrue(superstructure.armVoltPos()).onFalse(superstructure.arm2Home());
    //controller.operator.rightTrigger(.90).whileTrue(superstructure.armVoltPosManual(() -> controller.operator.getRightY()));
    //controller.operator.leftBumper().whileTrue(superstructure.armVoltVelManual(() -> controller.operator.getLeftY())).onFalse(superstructure.stopRollers());
    //controller.operator.b().whileTrue(superstructure.armVoltVel());
    //controller.operator.leftBumper().onTrue(superstructure.testElevator()).onFalse(superstructure.goToL0());
    controller.operator.rightTrigger(.90).whileTrue(superstructure.testElevatorManual(() -> controller.operator.getRightY()));
    











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


  public void setCoast() {
    superstructure.setPivotCoast();
  }

  public void setBrake() {
    //superstructure.setPivotBrake();
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   // return autoChooser.get();
    //return autoChooser.getSelected();
    //return null;
    double direction = 1; 
    var myAlliance = DriverStation.getAlliance();
    if (myAlliance.get() == Alliance.Blue) { return drivetrain.applyRequest(() -> 
    drive.withVelocityX(-0.5 * 1 * MaxSpeed) // Drive forward with negative Y (forward)
      .withVelocityY(0 * MaxSpeed) // Drive left with negative X (left)
      .withRotationalRate(0 * MaxAngularRate)).withTimeout(1).andThen(
            drivetrain.applyRequest( () ->   
            drive.withVelocityX(0 * MaxSpeed) // Drive forward with negative Y (forward)
              .withVelocityY(0 * MaxSpeed) // Drive left with negative X (left)
              .withRotationalRate(0 * MaxAngularRate)));
    }
    if (myAlliance.get() == Alliance.Red) { return drivetrain.applyRequest(() -> 
    drive.withVelocityX(-0.5 * -1 * MaxSpeed) // Drive forward with negative Y (forward)
      .withVelocityY(0 * MaxSpeed) // Drive left with negative X (left)
      .withRotationalRate(0 * MaxAngularRate)).withTimeout(1).andThen(
            drivetrain.applyRequest( () ->   
            drive.withVelocityX(0 * MaxSpeed) // Drive forward with negative Y (forward)
              .withVelocityY(0 * MaxSpeed) // Drive left with negative X (left)
              .withRotationalRate(0 * MaxAngularRate)));   
  } else { return null; }
  }

}
