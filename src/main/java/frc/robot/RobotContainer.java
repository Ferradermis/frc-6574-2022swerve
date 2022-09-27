// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.AutoSwerve;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.ShooterCommands.BasicShooterCycle;
import frc.robot.commands.ShooterCommands.SetHoodToPosition;
import frc.robot.commands.ShooterCommands.ZeroHood;
import frc.robot.commands.ShooterCommands.closedLoopShooterCycle;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final Joystick driver = new Joystick(0);
  //private final Joystick operator = new Joystick(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton driverYButton = new JoystickButton(driver, XboxController.Button.kY.value);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  public static final OI oi = new OI(); //Phase out
	public static final Shooter shooter = new Shooter();
	public static final Intake intake = new Intake();
  public static final ShooterHood hood = new ShooterHood();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true;
    boolean openLoop = true;
    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));

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

    /* Driver Buttons */
    driverYButton.whenPressed(new InstantCommand(() -> s_Swerve.zeroGyro()));
		//oi.driver_rightBumper.toggleWhenPressed(new IntakeProcess());
    oi.driver_aButton.whileHeld(new SetHoodToPosition(-4500)).whenReleased(()->hood.stop());
    oi.driver_rightTrigger.whileHeld(new BasicShooterCycle()).whenReleased(()->shooter.stopShootProcess());
    oi.driver_leftTrigger.whileHeld (new closedLoopShooterCycle()).whenReleased(()->shooter.stopShootProcess());
    oi.driver_bButton.whileHeld(new ZeroHood()).whenReleased(()->hood.stop());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    boolean fieldRelative = true;
    boolean openLoop = true;

    return new AutoSwerve(s_Swerve, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop).withTimeout(2);
  }
}
