package frc.team5973.robot.swerveBot.commands;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team5973.robot.subsystems.Limelight;
import frc.team5973.robot.subsystems.SwerveDrive;
import frc.team5973.robot.subsystems.SwerveDrive.Axis;
import frc.team5973.robot.subsystems.SwerveDrive.DriveMode;

public class SwerveDriveDefaultCommand extends CommandBase {
    final SwerveDrive drive;
    final Limelight limelight;
    final Map<Axis, DoubleSupplier> axisMap;
    final Map<DriveMode, BooleanSupplier> buttonMap;

    private final double DEADBAND;

    private final double SPEED_NORMAL;
    private final double SPEED_SAFE;

    private double speed;
 
    private double comboStartTimeSafe  = 0;
    private double comboStartTimeField = 0;
    private double comboStartTimeGoal  = 0;
    
    private boolean alreadyToggledSafeMode  = false;
    private boolean alreadyToggledFieldMode = false;
    private boolean alreadyToggledGoalMode  = false;

    private boolean safeMode          = false;
    private boolean fieldOrientedMode = false;
    private boolean goalOrientedMode  = false;

    private double buttonDelay = 0.0;

    private double forward;
    private double strafe;
    private double rotate;

    public double yawCorrection;
   
    private int driveMode = 1;

    private final int fieldOriented = 1;
    private final int robotOriented = 2;
    private final int goalOriented  = 3;


    public SwerveDriveDefaultCommand(final SwerveDrive drive,
                                     final Limelight limelight,
                                     final double DEADBAND,
                                     final double SPEED_NORMAL,
                                     final double SPEED_SAFE,
                                     final Map<Axis, DoubleSupplier> axisMap,
                                     final Map<DriveMode, BooleanSupplier> buttonMap) {
        
        this.limelight     = limelight;
        this.buttonMap     = buttonMap;
        this.axisMap       = axisMap;
        this.drive         = drive;
        
        this.DEADBAND = DEADBAND;
        this.SPEED_NORMAL = SPEED_NORMAL;
        this.SPEED_SAFE   = SPEED_SAFE;
    
        addRequirements(drive);
        SendableRegistry.addChild(SwerveDriveDefaultCommand.this, this);

    }

    @Override
	public void execute() {
        
        //determine whether to use safe mode or not
        speed = safeMode ? SPEED_SAFE : SPEED_NORMAL;
        
        //controller inputs
        forward =   MathUtil.clamp(MathUtil.applyDeadband(axis(Axis.FORWARD), DEADBAND) * speed, -1, 1);
        strafe  =  -MathUtil.clamp(MathUtil.applyDeadband(axis(Axis.STRAFE),  DEADBAND) * speed, -1, 1);
        rotate  =  -MathUtil.clamp(MathUtil.applyDeadband(axis(Axis.TURN),    0.1) * speed, -1, 1);

        if(driveMode == goalOriented) {
            yawCorrection = 0;
        } else if (driveMode == robotOriented || driveMode == fieldOriented) {
            yawCorrection = drive.correctHeading(0.004, forward, strafe, rotate);

        }
        

        //puts robot into safemode where the robot will go slower
         if(button(DriveMode.SAFEMMODE)) {

            if(comboStartTimeSafe == 0)
                comboStartTimeSafe = Timer.getFPGATimestamp();
            else if(Timer.getFPGATimestamp() - comboStartTimeSafe >= 3.0 && !alreadyToggledSafeMode) {
            
                safeMode = !safeMode;
                
                alreadyToggledSafeMode = true;
                System.out.println("Safemode is " + (safeMode ? "Enabled" : "Disabled") + ".");
            
            }

        } else {

            comboStartTimeSafe = 0;
            alreadyToggledSafeMode = false;
        
        }

        // toggle POV and field mode
        if(button(DriveMode.FIELDMODE) && !button(DriveMode.GOALMODE)) {

            if(comboStartTimeField == 0) {
                comboStartTimeField = Timer.getFPGATimestamp();
            } else if(Timer.getFPGATimestamp() - comboStartTimeField >= buttonDelay && !alreadyToggledFieldMode) {
                
                fieldOrientedMode = !fieldOrientedMode;

                alreadyToggledFieldMode = true;
                driveMode = fieldOrientedMode ? fieldOriented : robotOriented;
                System.out.println("Switching to " + (fieldOrientedMode ? "Field Oriented" : "Robot POV") + ".");
            }

        } else {

            comboStartTimeField = 0;
            alreadyToggledFieldMode = false;
            
        }

        //toggle goal centric mode
        if(button(DriveMode.GOALMODE) && !button(DriveMode.FIELDMODE) && limelight.isTargetValid()) {
            
            if(comboStartTimeGoal == 0) {
                comboStartTimeGoal = Timer.getFPGATimestamp();
            } else if(Timer.getFPGATimestamp() - comboStartTimeGoal >= buttonDelay && !alreadyToggledGoalMode) {
                goalOrientedMode = !goalOrientedMode;

                alreadyToggledGoalMode = true;
                driveMode = goalOrientedMode ? goalOriented : fieldOriented;
                System.out.println("Switching to " + (goalOrientedMode ? "Goal Oriented" : "Field Oriented") + ".");
            }
            
        } else if (button(DriveMode.GOALMODE) && !button(DriveMode.FIELDMODE) && !limelight.isTargetValid()) {

            comboStartTimeGoal = 0;
            alreadyToggledGoalMode = false;
            
            driveMode = fieldOriented;
            System.out.println("No valid target to change drive mode" + "\n Switching to Field Oriented Mode");
        } else {

            comboStartTimeGoal = 0;
            alreadyToggledGoalMode = false;
        }

        if(button(DriveMode.ZERO_GYRO)) {
            drive.resetGyro();

            yawCorrection = 0;

            System.out.println("Gyro reset");
        }

        //set drive modes
        switch (driveMode) {
            case fieldOriented: drive.swerveDrive(forward, strafe, rotate - yawCorrection, true);
                    break;
            case robotOriented: drive.swerveDrive(forward, strafe, rotate - yawCorrection, false);
                    break;
            case goalOriented: drive.swerveDrive(forward, 
                                                 strafe, 
                                                 rotate - limelight.limelightXPID(), 
                                                 true);
                    break;
            default: drive.swerveDrive(forward, strafe, rotate - yawCorrection, true);
                    break;
        }

       
	
	}

	private final double axis(Axis axis) {
		return axisMap.get(axis).getAsDouble();
	}

    private final boolean button(DriveMode button) {
        return buttonMap.get(button).getAsBoolean();
    }

	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);

        builder.addDoubleProperty("hello", () -> yawCorrection, null);

        
		
	}

}
