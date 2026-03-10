package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.hawklib.hid.XboxController;
import frc.robot.commands.SwerveCommand;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private enum InputScale {
        STANDARD,
        SQUARED,
        CUBED
    }

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final XboxController ctlDriver = new XboxController(0)
        .configAxisDeadzone(0.1)
        .configXAxisInverted(true)
        .configYAxisInverted(true);

   /* Driver Controls */
	private final int translationAxis = 1;
	private final int strafeAxis = 0;
	private final int rotationAxis = 4;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    private final JoystickButton dampen = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    private final JoystickButton DynamicLock = new JoystickButton(driver, XboxController.Button.kX.value);

    private final Trigger forwardHold = new Trigger(() -> (driver.getRawAxis(4) > 0.75));
    private final Trigger backwardHold = new Trigger(() -> (driver.getRawAxis(4) < -0.75));


    //Replacement Triggers
    private final Trigger btnZeroGyro = new Trigger(ctlDriver::getYButton);
    private final Trigger btnRobotCentric = new Trigger(ctlDriver::getLeftBumperButton);
    private final Trigger btnForwardHold = new Trigger(ctlDriver::getLeftTriggerButton);
    private final Trigger btnBackwardHold = new Trigger(ctlDriver::getRightTriggerButton);
    private final Trigger btnDynamicLock = new Trigger(ctlDriver::getXButton);
    private final Trigger btnDampen = new Trigger(ctlDriver::getRightBumperButton);

    private final Supplier<Double> axsDriver_Translation = () -> scaleInput(ctlDriver.getLeftY(), InputScale.CUBED);
    private final Supplier<Double> axsDriver_Strafe = () -> scaleInput(ctlDriver.getLeftX(), InputScale.CUBED);
    private final Supplier<Double> axsDriver_Rotation = () -> scaleInput(ctlDriver.getRightX(), InputScale.CUBED);

    /* Subsystems */
    private final PoseEstimator s_PoseEstimator = new PoseEstimator();
    private final Swerve s_Swerve = new Swerve(s_PoseEstimator);
    //private final Vision s_Vision = new Vision(s_PoseEstimator);

    /* AutoChooser */
    private final SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new SwerveCommand(
                s_Swerve, 
                () -> axsDriver_Translation.get(), 
                () -> axsDriver_Strafe.get(), 
                () -> axsDriver_Rotation.get(), 
                () -> !btnRobotCentric.getAsBoolean(),
                () -> !btnDampen.getAsBoolean(),
                () -> 0 // Dynamic heading placeholder
            )
        );

        // Configure the button bindings
        configureButtonBindings();


        //Pathplanner commands - templates
        NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
        NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
        NamedCommands.registerCommand("print hello", Commands.print("hello"));
    
        
        //Auto chooser
        autoChooser = AutoBuilder.buildAutoChooser("New Auto"); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    //TODO: Comments for scaleInput()
    private double scaleInput(double inputValue, InputScale scale) {
        switch(scale){
            case SQUARED:
                int sign = inputValue < 0.0 ? -1 : 1;
                return inputValue * inputValue * sign;
            case CUBED:
                return inputValue * inputValue * inputValue;
            default:
                return inputValue;
        }
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        btnZeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

    //Heading lock bindings
        btnForwardHold.onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.forwardHold)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
        );
        btnBackwardHold.onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.backwardHold)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
        );
        btnDynamicLock.onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.DynamicLock)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoChooser.getSelected();
    }
}