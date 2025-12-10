package utils;

public interface ShooterInterface {
    /**
     * should spin the flywheels at the passed in speed
     * @param speed the speed at which the flywheels should spin
     */
    public void spin(double speed);

    /**
     * should set the motor output to zero
     */
    public void stop();
}
