package frc.robot.math;

public class Vector extends Matrix {

	public Vector(int xSize) {
		super(xSize, 1);
	}

	public Vector(double... vector) {
		super(vector.length, 1);
		for (int i = 0; i < vector.length; i++) {
			getMat()[i][0] = vector[i];
		}
	}

	public Vector scale(double factor) {
		for (int x = 0; x < xSize; x++) {
			getMat()[x][0] *= factor;

		}
		return this;
	}

	public static Vector multiply(Matrix a, Matrix b) {
		if (a.ySize != b.xSize && b.ySize > 1) {
			return null;
		}
		Vector result = new Vector(a.xSize);
		for (int x = 0; x < result.xSize; x++) {
			for (int z = 0; z < a.ySize; z++) {
				result.getMat()[x][0] += a.getMat()[x][z] * b.getMat()[z][0];
			}
		}
		return result;
	}

	public double get(int row) {
		return getMat()[row][0];
	}
}
