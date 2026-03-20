package math;

public class PIDFController {
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;
    public static double nominalVoltage = 12.0;

    private double error = 0.0;
    private double integral = 0.0;
    private double derivative = 0.0;
    private double lastError = 0.0;
    private double MAX_INTEGRAL = 5000.0;

    private double output = 0.0;

    public double calculate(double target, double current, double currentVoltage) {
        error = target - current;
        integral += error;
        if (Math.abs(integral) > MAX_INTEGRAL) {
            integral = MAX_INTEGRAL * Math.signum(integral);
        }
        derivative = (error + lastError);
        output = kP * error + kI * integral + kD * derivative + kF * target + (error / Math.abs(error)) * (currentVoltage / nominalVoltage);
        lastError = error;

        if (Math.abs(output) > 1) {
            output = 1 * Math.signum(output);
        }
        return output;
    }

    public void resetController() {
        integral = 0.0;
        derivative = 0.0;
    }
}
