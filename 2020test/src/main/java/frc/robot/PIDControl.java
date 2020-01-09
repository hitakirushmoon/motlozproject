package frc.robot;

public class PIDControl {
	double kP, kI, kD;

	public PIDControl(double kP, double kI, double kD) {
		super();
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
	}

	double I = 0;
	double previous_error = Double.POSITIVE_INFINITY;

	double output(double error) {
		double P = error * kP;
		if (P < 0.5) {
		I += error * kI * Constants.loopTime;
		}
		double D = previous_error == Double.POSITIVE_INFINITY ? 0 : (error - previous_error) * kD / Constants.loopTime;
		previous_error = error;
		return P + I + D;
	}
}
