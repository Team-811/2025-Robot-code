// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.setSpeedCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private static final double slowSpeed = 0.3;
    
        private static final double fastSpeed = 1;
        
            private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)* OperatorConstants.kSpeed; // kSpeedAt12Volts desired top speed
            private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
        
            /* Setting up bindings for necessary control of the swerve drive platform */
            private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
            private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
            private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        
            private final Telemetry logger = new Telemetry(MaxSpeed);
              private SlewRateLimiter slewwyY = new SlewRateLimiter(1.5);
          private SlewRateLimiter slewwyX = new SlewRateLimiter(1.5);
        
            private final CommandXboxController driveController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
            private final CommandXboxController OpController = new CommandXboxController(OperatorConstants.kOpControllerPort);
            public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        
        //   private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
        //       .withDeadband(MaxSpeed*0.05).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        //       .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want robot-centric
            
          private final SwerveRequest.FieldCentricFacingAngle driveFacing = new SwerveRequest.FieldCentricFacingAngle()
              .withDeadband(MaxSpeed * 0.05).withDriveRequestType(DriveRequestType.OpenLoopVoltage);
          private final PhoenixPIDController steerController = new PhoenixPIDController(3, 0, 0.05);
        //   private final PhoenixPIDController steerController180 = new PhoenixPIDController(0.3, 0.001, 0.01);
        //   private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
          // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        
            public RobotContainer() {
                configureBindings();
        
            }
        
            private void configureBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                driveFacing.HeadingController = steerController;
                drivetrain.setDefaultCommand(
                    // Drivetrain will execute this command periodically
                    drivetrain.applyRequest(() ->
                        drive.withVelocityX(joyLeftY()* MaxSpeed) // Drive forward with negative Y (forward)
                            .withVelocityY(-joyLeftX() * MaxSpeed) // Drive left with negative X (left)
                            .withRotationalRate(-joyRightX()* MaxAngularRate) // Drive counterclockwise with negative X (left)
                    )
                );
        
                driveController.a().whileTrue(drivetrain.applyRequest(() -> brake));
                driveController.b().whileTrue(drivetrain.applyRequest(() ->
                    point.withModuleDirection(new Rotation2d(-driveController.getLeftY(), -driveController.getLeftX()))
                ));
                driveController.rightBumper().whileTrue(new setSpeedCommand(slowSpeed));
            driveController.leftBumper().whileTrue(new setSpeedCommand(fastSpeed));
    
            
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driveController.back().and(driveController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driveController.back().and(driveController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driveController.start().and(driveController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driveController.start().and(driveController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        // driveController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivetrain.registerTelemetry(logger::telemeterize);                                                                    

    }
    

    
    public double joyRightX(){
        double rightX = driveController.getRightX();
        if(Math.abs(rightX)> OperatorConstants.kJoyRightXDeadzone){
            return rightX;
        }
        return 0;
    }
    public double joyLeftX(){
        double leftX = driveController.getLeftX();
        if(Math.abs(leftX) > OperatorConstants.kJoyLeftXDeadzone){
            return leftX;
        }
        return 0;
    }
    public double joyLeftY(){
        double  leftY= driveController.getLeftY();
        if(Math.abs(leftY) > OperatorConstants.kJoyLeftYDeadzone){
            return leftY;
        }
        return 0;
    }


    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
