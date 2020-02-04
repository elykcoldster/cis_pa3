package cis.pa3.tools;
import java.util.List;

import Jama.Matrix;

/**
 * A column vector in n-th dimensional space
 * 
 * @author John Lee, Kyle Xiong
 *
 */
public class ColumnVector {
	private Matrix vector;

	/**
	 * Constructs an empty n-dimensional column vector
	 * @param dimension
	 */
	public ColumnVector(int dimension) {
		this.vector = new Matrix(dimension, 1);
	}

	/**
	 * Constructs a column vector out of a matrix
	 * @param coordinates
	 */
	public ColumnVector(Matrix coordinates) {
		if (coordinates.getColumnDimension() != 1) {
			throw new IllegalArgumentException(
			        "Argument must be a column vector");
		}
		this.vector = coordinates;
	}

	/**
	 * Constructs a column vector out of a double array
	 * @param coordinates
	 */
	public ColumnVector(double[] coordinates) {
		this.vector = new Matrix(coordinates.length, 1);
		this.setVector(coordinates);
	}


	/**
	 * Performs vector addition on two vectors in the form:
	 * a = a + b
	 * @param vector
	 * @return This vector after adding
	 */
	public ColumnVector plusEquals(ColumnVector vector) {
		this.vector = this.vector.plus(vector.getMatrix());
		return this;
	}

	/**
	 * Performs vector addition on two vectors in the form:
	 * c = a + b
	 * @param vector
	 * @return The sum of the two vectors
	 */
	public ColumnVector plus(ColumnVector vector) {
		return new ColumnVector(this.vector.plus(vector.getMatrix()));
	}
	
	/**
	 * Performs vector subtraction on two vectors in the form:
	 * a = a - b
	 * @param vector
	 * @return This vector after subtracting
	 */
	public ColumnVector minusEquals(ColumnVector vector) {
		this.vector = this.vector.minus(vector.getMatrix());
		return this;
	}
	
	/**
	 * Performs vector subtraction on two vectors in the form:
	 * c = a - b
	 * @param vector
	 * @return The difference of the two vectors
	 */
	public ColumnVector minus(ColumnVector vector) {
		return new ColumnVector(this.vector.minus(vector.getMatrix()));
	}

	/**
	 * Calculates the outer product of two vectors in the order:
	 * (this)x(vector)
	 * @param vector 
	 * @return The outer product matrix
	 */
	public Matrix outerProduct(ColumnVector vector) {
		checkDimensions(vector.getMatrix());
		return this.vector.times(vector.getMatrix().transpose());
	}
	
	/**
	 * Performs scalar multiplication on this column vector
	 * @param scalar The scalar value
	 * @return This vector after the multiplication
	 */
	public ColumnVector timesEquals(double scalar) {
		this.vector.times(scalar);
		return this;
	}
	
	/**
	 * Returns the results of scalar multiplication on this vector
	 * @param scalar The scalar value
	 * @return A new vector with the results
	 */
	public ColumnVector times(double scalar) {
		return new ColumnVector(this.vector.times(scalar));
	}
	
	/**
	 * Takes an inner product between two vectors in the form:
	 * c = a.*b
	 * @param vector The vector to multiply with
	 * @return The scalar result
	 */
	public double innerProduct(ColumnVector vector) {
		this.checkDimensions(vector.getMatrix());
		double sum = 0;
		for (int i = 0; i < this.getDimension(); i++) {
			sum += this.get(i) + vector.get(i);
		}
		return sum;
	}

	/**
	 * Returns the skew (or cross product matrix) of a 3D vector.
	 * If not a 3D vector, then an exception is thrown
	 * @return The cross-product matrix
	 */
	public Matrix skew() {
		if (this.getDimension() != 3) {
			throw new IllegalStateException(
			        "Can only compute skew of 3D vector");
		}
		double[] a = {this.vector.get(0, 0), this.vector.get(1, 0),
		        this.vector.get(2, 0)};
		double[][] skewMatrix = {
				{0, -1 * a[2], a[1]}, 
				{a[2], 0, -1 * a[0]},
		        {-1 * a[1], a[0], 0}};
		return new Matrix(skewMatrix);
	}
	
	/**
	 * Returns a new unit vector from this vector
	 * @return A new ColumnVector that is a unit vector of this
	 */
	public ColumnVector unitVector() {
		return new ColumnVector(this.vector.times(1/this.vector.normF()));
	}
	
	/**
	 * Normalizes this vector using the Frobenius norm
	 * @return This ColumnVector as a unit vector
	 */
	public ColumnVector normalize() {
		this.vector = this.vector.times(1/this.magnitude());
		return this;
	}
	
	/**
	 * Returns the magnitude of this vector
	 * @return The magnitude value
	 */
	public double magnitude() {
		return this.vector.normF();
	}

	/**
	 * Converts a Matrix that is in the form of a column vector
	 * into a ColumnVector object
	 * @param coordinates
	 */
	public void setVector(Matrix coordinates) {
		if (!this.checkDimensions(coordinates)) {
			throw new IllegalArgumentException(
			        "Vectors must have matching dimensions");
		}
	
		this.vector = coordinates;
	}

	/**
	 * Sets the vector field as a column vector of the given coordinates
	 * @param coordinates
	 */
	public void setVector(double... coordinates) {
		if (!this.checkDimensions(coordinates)) {
			throw new IllegalArgumentException(
			        "Vectors must have matching dimensions");
		}
		for (int i = 0; i < coordinates.length; i++) {
			this.vector.set(i, 0, coordinates[i]);
		}
	}

	public double get(int i) {
		return this.vector.get(i, 0);
	}

	/**
	 * Returns the Matrix representation of this column vector
	 * @return The Matrix of this vector
	 */
	public Matrix getMatrix() {
		return this.vector;
	}

	/**
	 * Returns the dimensions of this column vector
	 * @return The dimension value
	 */
	public int getDimension() {
		return this.vector.getRowDimension();
	}

	private boolean checkDimensions(Matrix a) {
		return a.getRowDimension() == this.vector.getRowDimension()
		        && a.getColumnDimension() == this.vector.getColumnDimension();
	}

	private boolean checkDimensions(double[] a) {
		return a.length == this.vector.getRowDimension();
	}

	@Override
	public boolean equals(Object o) {
		if (o == null) {
			return false;
		}

		if (this.getClass() != o.getClass()) {
			return false;
		}
		
		if (((ColumnVector) o).getDimension() != this.getDimension()) {
			return false;
		}
		Matrix otherVector = ((ColumnVector) o).getMatrix();

		for (int i = 0; i < this.getDimension(); i++) {
			if (otherVector.get(i, 0) != this.vector.get(i, 0)) {
				return false;
			}
		}
		return true;
	}
	public static Matrix ColumnVectorListToMatrix(List<ColumnVector> lc) {
		int numrows = lc.get(0).getMatrix().getRowDimension();
		int numcols = lc.size();
		Matrix M = new Matrix(numrows, numcols);
		for (int i = 0; i < numcols; i++) {
			M.setMatrix(0, numrows - 1, i, i, lc.get(i).getMatrix());
		}
		return M;
	}
}