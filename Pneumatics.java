package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Pneumatics {
    
    Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.REVPH);
    DoubleSolenoid doubleSolenoid1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 2);
    DoubleSolenoid doubleSolenoid2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 5);

    AnalogPotentiometer potentiometer = new AnalogPotentiometer(0, 90, 90);


    public Pneumatics(){
        doubleSolenoid1.set(Value.kOff);
    }

    public void runForward(){
        doubleSolenoid1.set(Value.kForward);
        doubleSolenoid2.set(Value.kReverse);
    }

    public void runReverse(){
        doubleSolenoid1.set(Value.kReverse);
        doubleSolenoid2.set(Value.kForward);
    }

    public void middle(){
        double angle = potentiometer.get() * 90;

        if (angle < 39 && angle > 0){

            System.out.println(1.0);
            doubleSolenoid1.set(Value.kForward);
            doubleSolenoid2.set(Value.kReverse);

        }

        if (angle > 51 && angle < 90){

            System.out.println(2.0);
            doubleSolenoid1.set(Value.kReverse);
            doubleSolenoid2.set(Value.kForward);

        }

        if (angle > 40 && angle < 50){

            System.out.println(3.0);
            doubleSolenoid1.set(Value.kOff);

        }
    }

    public double getPotentiometerAngle(){
        return potentiometer.get();
    }

}
