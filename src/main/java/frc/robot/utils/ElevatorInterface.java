package frc.robot.utils;

public interface ElevatorInterface {
    /**
     * sets the desired position for the elevator
     * @param setpoint the value that the elevator should go to
     */
    public void setSetpoint(double setpoint);

    /**
     * should set the elevators output to the feed forward value to hold the elevator in place
     */
    public void hover();

    /**
     * should set the elevator output to the speed variable
     * @param speed the speed the elevator should move
     */
    public void move(double speed);

    /**
     * should zero either an absolute encoder or the motor's relative encoder
     */
    public void zeroEncoders();
}
