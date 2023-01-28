package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Stabilize extends CommandBase{

    public Stabilize(){


        
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        DrivetrainSubsystem.navX.reset();
        SmartDashboard.putNumber("Gyroscope Initalize", 0);
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double xRotation = DrivetrainSubsystem.navX.getRoll();

        System.out.println("Pitch: " + xRotation);
        System.out.println("Yaw: " + DrivetrainSubsystem.navX.getYaw());
        System.out.println("Roll: " + DrivetrainSubsystem.navX.getRoll());


        if (xRotation < 45 && xRotation > 5){

            DrivetrainSubsystem.leftFrontMotor.set(0.2);
            DrivetrainSubsystem.leftBackMotor.set(0.2);
            DrivetrainSubsystem.rightFrontMotor.set(-0.2);
            DrivetrainSubsystem.rightBackMotor.set(-0.2);

        }

        if (xRotation > -45 && xRotation < -5){

            DrivetrainSubsystem.leftFrontMotor.set(0.2);
            DrivetrainSubsystem.leftBackMotor.set(0.2);
            DrivetrainSubsystem.rightFrontMotor.set(0.2);
            DrivetrainSubsystem.rightBackMotor.set(0.2);

        }

        if (xRotation > -3 && xRotation < 3){

            DrivetrainSubsystem.leftFrontMotor.set(0);
            DrivetrainSubsystem.leftBackMotor.set(0);
            DrivetrainSubsystem.rightFrontMotor.set(0);
            DrivetrainSubsystem.rightBackMotor.set(0);

        }

      }
    
      // Called once the command ends or is interrupted.
      @Override
      public void end(boolean interrupted) {}
    
      // Returns true when the command should end.
      @Override
      public boolean isFinished() {
        return false;
      }

}
