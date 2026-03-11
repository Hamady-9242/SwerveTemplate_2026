package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.hawklib.hid.XboxController;
import frc.robot.Constants.DriverConstants;
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

    /* Controllers */
    private final XboxController ctlDriver = new XboxController(0)
        .configAxisDeadzone(0.1)
        .configXAxisInverted(true)
        .configYAxisInverted(true);


    //Driver Button Mapping
    private final Trigger btnZeroGyro = new Trigger(ctlDriver::getStartButton)
                                            .and(ctlDriver::getBackButton);
    private final Trigger btnFieldCentric = new Trigger(ctlDriver::getLeftBumperButton);

    private final Trigger btnDriver_HighSpeed = new Trigger(ctlDriver::getRightBumperButton);

    //Driver Axis Mapping
    private final Supplier<Double> axsDriver_Translation = () -> squareInput(ctlDriver.getLeftY());
    private final Supplier<Double> axsDriver_Strafe = () -> squareInput(ctlDriver.getLeftX());
    private final Supplier<Double> axsDriver_Rotation = () -> squareInput(ctlDriver.getRightX());

    private final Supplier<Double> SPEED_MULT = () -> (btnDriver_HighSpeed.getAsBoolean() ? DriverConstants.HIGH_SPEED : DriverConstants.SLOW_SPEED);

    /* Subsystems */
    private final PoseEstimator s_PoseEstimator = new PoseEstimator();
    private final Swerve CHASSIS = new Swerve(s_PoseEstimator);
    //private final Vision s_Vision = new Vision(s_PoseEstimator);

    /* AutoChooser */
    private final SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        CHASSIS.setDefaultCommand(
            new SwerveCommand(
                CHASSIS, 
                () -> axsDriver_Translation.get() * SPEED_MULT.get(), 
                () -> axsDriver_Strafe.get() * SPEED_MULT.get(), 
                () -> axsDriver_Rotation.get() * SPEED_MULT.get(), 
                () -> !btnFieldCentric.getAsBoolean(),
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

    @SuppressWarnings("unused")
    private double squareInput(double inputValue) {
        int sign = inputValue < 0.0 ? -1 : 1;
        return inputValue * inputValue * sign;
    }

    @SuppressWarnings("unused")
    private double cubeInput(double inputValue) {
        return inputValue * inputValue * inputValue;
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        btnZeroGyro.onTrue(new InstantCommand(() -> CHASSIS.zeroHeading()));
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