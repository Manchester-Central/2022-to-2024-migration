// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.chaos131.gamepads.Gamepad;
import com.chaos131.auto.AutoBuilder;
import com.chaos131.auto.ParsedCommand;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AimToGoal;
import frc.robot.commands.AutoAimLaunch;
import frc.robot.commands.CameraDefault;
import frc.robot.commands.EnableSlowDriverSpeed;
import frc.robot.commands.ClimberDefault;
import frc.robot.commands.DashboardSpeedLauncherShoot;
import frc.robot.commands.DriveOverDistance;
import frc.robot.commands.DriveOverTime;
import frc.robot.commands.DriveToPosition;
import frc.robot.commands.DriverRelativeDrive;
import frc.robot.commands.DriverRelativeDriveAimAndLaunch;
import frc.robot.commands.DriverRelativeDriveWithAim;
import frc.robot.commands.FieldRelativeDrive;
import frc.robot.commands.HandBrake;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeDefault;
import frc.robot.commands.IntakeWithOnlyFeeder;
import frc.robot.commands.LauncherDefault;
import frc.robot.commands.CameraLauncherShoot;
import frc.robot.commands.Output;
import frc.robot.commands.FeederDefault;
import frc.robot.commands.RobotRelativeDrive;
import frc.robot.commands.SetSpeedLauncherShoot;
import frc.robot.commands.SetFeederMode;
import frc.robot.commands.SwerveModuleTest;
import frc.robot.commands.SwerveMotorTest;
import frc.robot.commands.ZeroNavX;
import frc.robot.commands.auto.AutoDriverRelativeDrive;
import frc.robot.commands.auto.AutoOutput;
import frc.robot.commands.auto.AutoRobotRelativeDrive;
import frc.robot.commands.auto.LoadBalls;
import frc.robot.commands.auto.StartingPosition;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.FlywheelTable;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Feeder.FeederMode;
import frc.robot.subsystems.SwerveDrive.SwerveModulePosition;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private Gamepad m_driver = new Gamepad(0);
  private Gamepad m_operator = new Gamepad(1);
  // private final ExampleCommand m_autoCommand = new
  // ExampleCommand(m_exampleSubsystem);

  public SwerveDrive m_swerveDrive = new SwerveDrive();
  private Climber m_climber = new Climber();
  private Camera m_camera = new Camera();
  private Launcher m_launcher = new Launcher();
  private Feeder m_feeder = new Feeder();
  private FlywheelTable m_flywheelTable = new FlywheelTable();
  private Intake m_intake = new Intake();

  private AutoBuilder m_autoBuilder = new AutoBuilder();
  public PowerDistribution m_pdp = new PowerDistribution();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_autoBuilder.registerCommand("robotRelativeDrive",
        (ParsedCommand pc) -> new AutoRobotRelativeDrive(pc, m_swerveDrive));
    m_autoBuilder.registerCommand("driverRelativeDrive",
        (ParsedCommand pc) -> new AutoDriverRelativeDrive(pc, m_swerveDrive));
    m_autoBuilder.registerCommand("launch",
        (ParsedCommand pc) -> new CameraLauncherShoot(m_launcher, m_camera, m_feeder, m_flywheelTable));
    m_autoBuilder.registerCommand("launchNoCamera",
        (ParsedCommand pc) -> SetSpeedLauncherShoot.CreateAutoCommand(pc, m_launcher, m_feeder));
    m_autoBuilder.registerCommand("startingPosition", (ParsedCommand pc) -> new StartingPosition(pc, m_swerveDrive));
    m_autoBuilder.registerCommand("driveToPosition",
        (ParsedCommand pc) -> DriveToPosition.CreateAutoCommand(pc, m_swerveDrive));
    m_autoBuilder.registerCommand("aimToGoal", (ParsedCommand pc) -> new AimToGoal(m_swerveDrive, m_camera));
    m_autoBuilder.registerCommand("aimLaunch", (ParsedCommand pc) -> new AutoAimLaunch(m_swerveDrive, m_driver, m_camera, m_launcher, m_flywheelTable, m_feeder));
    m_autoBuilder.registerCommand("spit", (ParsedCommand pc) -> new AutoOutput(m_feeder, m_intake));
    m_autoBuilder.registerCommand("loadBalls", (ParsedCommand pc) -> new LoadBalls(pc, m_feeder));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Command driverRelativeDrive = new DriverRelativeDrive(m_swerveDrive, m_driver);

    // Default Commands
    m_swerveDrive.setDefaultCommand(driverRelativeDrive);
    m_climber.setDefaultCommand(new ClimberDefault(m_climber, m_operator));
    m_intake.setDefaultCommand(new IntakeDefault(m_intake));
    m_launcher.setDefaultCommand(new LauncherDefault(m_launcher));
    m_feeder.setDefaultCommand(new FeederDefault(m_feeder));
    m_camera.setDefaultCommand(new CameraDefault(m_camera));

    configureMatchCommands();
    // configureDebugCommands();

  }

  private void configureDebugCommands() {
    m_driver.b().whileTrue(new AimToGoal(m_swerveDrive, m_camera).repeatedly());
    m_driver.rightStick().whileTrue(new DriverRelativeDriveWithAim(m_swerveDrive, m_driver, m_camera).repeatedly());
  }

  private void configureMatchCommands() {

    Command driverRelativeDrive = new DriverRelativeDrive(m_swerveDrive, m_driver);
    Command robotRelativeDrive = new RobotRelativeDrive(m_swerveDrive, m_driver);

    // Drive Commands
    m_driver.back().onTrue(robotRelativeDrive);
    m_driver.start().onTrue(driverRelativeDrive);

    m_driver.leftStick().whileTrue(new EnableSlowDriverSpeed(true).repeatedly());
    m_driver.rightStick().whileTrue(new DriverRelativeDriveWithAim(m_swerveDrive, m_driver, m_camera).repeatedly());

    // m_driver.getButtonA().whenPressed(new EnableSlowDriverSpeed(true));
    // m_driver.getButtonB().whenPressed(new EnableSlowDriverSpeed(false));
    m_driver.y().whileTrue(new DashboardSpeedLauncherShoot(m_launcher, m_feeder).repeatedly());

    m_driver.leftBumper().whileTrue(new IntakeWithOnlyFeeder(m_feeder).repeatedly());
    m_driver.leftTrigger().whileTrue(new IntakeCommand(m_feeder, m_intake).repeatedly());

    m_driver.rightBumper()
        .whileTrue(new SetSpeedLauncherShoot(m_launcher, m_feeder, Constants.DefaultLauncherHighSpeed, FeederMode.LAUNCH_HIGH_BUMPER, Constants.DefaultLauncherTolerance));
    m_driver.rightTrigger()
        .whileTrue(new DriverRelativeDriveAimAndLaunch(m_swerveDrive, m_driver, m_camera, m_launcher, m_flywheelTable, m_feeder).repeatedly());

    m_driver.povUp().onTrue(new ZeroNavX(0, m_swerveDrive));
    m_driver.povRight().onTrue(new ZeroNavX(90, m_swerveDrive));
    m_driver.povDown().onTrue(new ZeroNavX(180, m_swerveDrive));
    m_driver.povLeft().onTrue(new ZeroNavX(270, m_swerveDrive));
    m_driver.x().whileTrue(new HandBrake(m_swerveDrive).repeatedly());

    // Operator Commands
    m_operator.a().whileTrue(new IntakeCommand(m_feeder, m_intake).repeatedly());
    m_operator.b().whileTrue(new RunCommand(() -> m_intake.ManualIntake(1.0), m_intake)
        .alongWith(new SetFeederMode(m_feeder, FeederMode.BOTTOM_ONLY)).repeatedly());
    m_operator.x().whileTrue(new SetFeederMode(m_feeder, FeederMode.LAUNCH_CAMERA).repeatedly());
    m_operator.y().whileTrue(new Output(m_feeder, m_intake).repeatedly());

    m_operator.rightBumper().whileTrue(new CameraLauncherShoot(m_launcher, m_camera, m_feeder, m_flywheelTable).repeatedly());
    m_operator.rightTrigger()
        .whileTrue(new SetSpeedLauncherShoot(m_launcher, m_feeder, Constants.DefaultLauncherLowSpeed,
         FeederMode.LAUNCH_LOW_BUMPER, Constants.DefaultLauncherToleranceLowBumper).repeatedly());

    m_operator.leftBumper().whileTrue(new IntakeWithOnlyFeeder(m_feeder).repeatedly());
    m_operator.leftTrigger().whileTrue(new IntakeCommand(m_feeder, m_intake).repeatedly());

    m_operator.povLeft().onTrue(new RunCommand(() -> m_climber.MoveArmUp(), m_climber).repeatedly());
    m_operator.povRight().onTrue(new RunCommand(() -> m_climber.MoveArmDown(), m_climber).repeatedly());
    m_operator.povUp().onTrue(new RunCommand(() -> m_climber.ExtendToTop(), m_climber).repeatedly());
    m_operator.povDown().onTrue(new RunCommand(() -> m_climber.ExtendToBottom(), m_climber).repeatedly());
  
    m_operator.start().whileTrue(new RunCommand(() -> m_launcher.MoveHoodUp(), m_launcher).repeatedly());
    m_operator.back().whileTrue(new RunCommand(() -> m_launcher.MoveHoodDown(), m_launcher).repeatedly());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    var spinUpLauncherStart = new InstantCommand(() -> m_launcher.spinUpSpeed(), m_launcher);
    return spinUpLauncherStart.andThen(m_autoBuilder.createAutoCommand());
  }
}
