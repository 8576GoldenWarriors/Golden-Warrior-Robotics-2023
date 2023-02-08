package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Stabilize extends CommandBase{
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final DrivetrainSubsystem drivetrainSubsystem;

    public Stabilize(){
    this.drivetrainSubsystem = new DrivetrainSubsystem();
    addRequirements(drivetrainSubsystem);
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

        float xRotation = DrivetrainSubsystem.navX.getRoll();
        System.out.println(xRotation);

        // System.out.println("Pitch: " + xRotation);
        // System.out.println("Yaw: " + DrivetrainSubsystem.navX.getYaw());
        // System.out.println("Roll: " + DrivetrainSubsystem.navX.getRoll());


        if (xRotation < 30 && xRotation > 5){

            System.out.println(1.0);
            drivetrainSubsystem.arcadeDrive(-2.0, 0.0);

        }

        if (xRotation > -30 && xRotation < -5){

            System.out.println(2.0);
            drivetrainSubsystem.arcadeDrive(2.0, 0.0);

        }

        if (xRotation > -3 && xRotation < 3){

            System.out.println(3.0);
            drivetrainSubsystem.arcadeDrive(0.0, 0.0);

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
