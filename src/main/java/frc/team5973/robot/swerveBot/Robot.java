package frc.team5973.robot.swerveBot;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.team5973.robot.RobotBase;
import frc.team5973.robot.XinputController;
import frc.team5973.robot.subsystems.Limelight;
import frc.team5973.robot.subsystems.SwerveDrive;
import frc.team5973.robot.subsystems.SwerveDrive.Axis;
import frc.team5973.robot.subsystems.SwerveDrive.DriveMode;
import frc.team5973.robot.swerveBot.commands.SwerveDriveDefaultCommand;

public class Robot extends RobotBase {
	
	private Limelight limelight;
	private SwerveDrive drive;

	private Command autonomousCommand;

	private int driverPort  = 0;
	private int copilotPort = 1;

	private final double SPEED_NORMAL  = 0.9;
	private final double SPEED_SAFE    = 0.3;
	
	private final double DEADBAND  = 0.07;

	private XinputController driver  = new XinputController(driverPort);
	private XinputController copilot = new XinputController(copilotPort);



	public Robot() {
		super("swerveBot");

		port("frontRightSpeedMotor", 3); //drive motor ports
        port("frontLeftSpeedMotor",  0);
        port("backRightSpeedMotor",  2);
        port("backLeftSpeedMotor",   1);
        
        port("frontRightAngleMotor", 7); //rotation motor ports
        port("frontLeftSAngleMotor", 4);
        port("backRightAngleMotor",  6);
        port("backLeftAngleMotor",   5);

		drive = new SwerveDrive(this);
		
		limelight = new Limelight(this);
	
		drive.setDefaultCommand(new SwerveDriveDefaultCommand(drive, 
															  limelight, 
															  DEADBAND,
															  SPEED_NORMAL, 
														      SPEED_SAFE, 
															  Map.of(
			Axis.FORWARD, () -> driver.getLeftY(),
			Axis.STRAFE,  () -> driver.getLeftX(),
			Axis.TURN,    () -> driver.getRightX()),
			Map.of(
			DriveMode.SAFEMMODE, () -> driver.getLeftStickButton() && driver.getRightStickButton(),
			DriveMode.FIELDMODE, () -> copilot.getBButton(),
			DriveMode.GOALMODE,  () -> driver.getAButton(),
			DriveMode.ZERO_GYRO, () -> copilot.getXButton()
			)));

	}

	@Override
	public void robotInit() {

	}

	@Override
	public void teleopInit() {
			
		if (autonomousCommand != null)
			autonomousCommand.cancel();
	}

	@Override
	public void teleopPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void testPeriodic() {
		CommandScheduler.getInstance().run();
		
	}

	@Override
	public void disabledInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
			autonomousCommand = null;
		}
	}

	@Override
	public void autonomousInit() {

		// schedule the autonomous command (example)
		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		  }
		
	}

	@Override
	public void autonomousPeriodic() {
		CommandScheduler.getInstance().run();
	}

}
