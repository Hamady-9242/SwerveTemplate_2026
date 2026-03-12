package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;


public class SwerveCommand extends Command {    
    private final Swerve sysSwerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private DoubleSupplier dynamicHeadingSup;
    private BooleanSupplier robotCentricSup;
    private PIDController rotationController;
    

    public SwerveCommand(Swerve sysSwerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, DoubleSupplier dynamicHeadingSup) {
        this.sysSwerve = sysSwerve;
        addRequirements(sysSwerve);

        rotationController = new PIDController(SwerveConstants.HeadingKP, SwerveConstants.HeadingKI, SwerveConstants.HeadingKD );
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(SwerveConstants.HeadingTolerence);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.dynamicHeadingSup = dynamicHeadingSup;
    }

    @Override
    @SuppressWarnings("unused")
    public void execute() {
        /* Get Values, Deadband, Dampen */
        double translationVal = translationSup.getAsDouble();
        double strafeVal = strafeSup.getAsDouble();
        double rotationVal = rotationSup.getAsDouble();
        //TODO: Add code for dynamic heading- the supplier is a placeholder right now
        double dynamicHeading = dynamicHeadingSup.getAsDouble();
        
     //heading lock - forward
     /*
        rotationVal = rotationController.calculate(s_Swerve.getHeading().getRadians(), Units.degreesToRadians(0));
        */
        rotationVal = rotationVal * SwerveConstants.maxAngularVelocity;

        /* Drive */
        sysSwerve.drive(
            new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxSpeed), 
            rotationVal,
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}