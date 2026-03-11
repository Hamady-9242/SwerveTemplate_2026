package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
//import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.hawklib.dashboard.DashboardValue;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {
    private PoseEstimator s_PoseEstimator = new PoseEstimator();

    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveModules;
    public Pigeon2 gyro;
    public RobotConfig config;
    private Field2d field = new Field2d();

    private NetworkTable tblSwerve = NetworkTableInstance.getDefault().getTable("Swerve");
    private NetworkTable tblModules = tblSwerve.getSubTable("Modules");

    private final String[] MODULE_NAMES = {"Front Left", "Front Right", "Back Left", "Back Right"};

    //private NetworkTableEntry dshGyro = tblSwerve.getEntry("Gyro");
    //private NetworkTableEntry dshHeading = tblSwerve.getEntry("Heading");

    @SuppressWarnings("unused")
    private final DashboardValue<Double> dshGyro = new DashboardValue<Double>(tblSwerve, "Gyro", () -> getGyroYaw().getDegrees());
    
    @SuppressWarnings("unused")
    private final DashboardValue<Double> dshHeading = new DashboardValue<Double>(tblSwerve, "Heading", () -> getHeading().getDegrees());

    public Swerve(PoseEstimator s_PoseEstimator) {
        this.s_PoseEstimator = s_PoseEstimator;

        config = new RobotConfig(
            Constants.AutoConstants.ROBOT_MASS_KG,
            Constants.AutoConstants.ROBOT_MOI,
            Constants.AutoConstants.moduleConfig,
            SwerveConstants.trackWidth);

        //FIXME: Move pigeon instance out of Swerve?
        gyro = new Pigeon2(SwerveConstants.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveModules = new SwerveModule[] {
            new SwerveModule(0, SwerveConstants.FrontLeftModule.constants),
            new SwerveModule(1, SwerveConstants.FrontRightModule.constants),
            new SwerveModule(2, SwerveConstants.BackLeftModule.constants),
            new SwerveModule(3, SwerveConstants.BackRightModule.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, getGyroYaw(), getModulePositions());
        System.out.println(getPose().getX());


        AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    Constants.AutoConstants.translationPID, // Translation PID constants
                    Constants.AutoConstants.rotationPID // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
    
        // Set up custom logging to add the current path to a field 2d widget
        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

        SmartDashboard.putData("Field", field);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            SwerveConstants.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation
                                )
            );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

        for(SwerveModule module: mSwerveModules){
            module.setDesiredState(swerveModuleStates[module.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by Pathplanner autobuilder */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxSpeed);
        
        for(SwerveModule module: mSwerveModules){
            module.setDesiredState(desiredStates[module.moduleNumber], true);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose); // First used to be getHeading()
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule module: mSwerveModules){
            states[module.moduleNumber] = module.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule module: mSwerveModules){
            positions[module.moduleNumber] = module.getPosition();
        }
        return positions;
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return SwerveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelative(ChassisSpeeds speeds){
        SwerveModuleState[] states = SwerveConstants.swerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.maxSpeed);
        setModuleStates(states);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule module : mSwerveModules){
            module.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getGyroYaw(), getModulePositions()); 
        s_PoseEstimator.updateSwerve(getGyroYaw(), getModulePositions());
        field.setRobotPose(getPose());
        
        for(SwerveModule module : mSwerveModules) {
            NetworkTable tblmodule= tblModules.getSubTable("Module [" + MODULE_NAMES[module.moduleNumber] + "]");
            tblmodule.getEntry("CANcoder").setDouble(module.getCANcoder().getDegrees());
            tblmodule.getEntry("Angle").setDouble(module.getPosition().angle.getDegrees());
        }
    }
}