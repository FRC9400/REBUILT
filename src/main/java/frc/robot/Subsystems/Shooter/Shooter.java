package frc.robot.Subsystems.Shooter;


import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    private final ShooterIO shooterIO;
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private double setpointVelocity;

    public Shooter(ShooterIO shooterIO){
        this.shooterIO = shooterIO;
        setpointVelocity = 0.0;
    }

    @Override
    public void periodic(){
        shooterIO.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    public void setVelocity(double velocity, double ratio) {
        setpointVelocity = velocity;
        shooterIO.setVelocity(setpointVelocity, ratio);
    }

    public void zeroVelocity() {
        setpointVelocity = 0;
        shooterIO.zeroVelocity();
    }

}
