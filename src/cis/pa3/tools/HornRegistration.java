package cis.pa3.tools;
import Jama.Matrix;

/**
 * An implementation of Horn's point-cloud-to-point-cloud registration algorithm
 * @author Kyle Xiong, John Lee
 *
 */
public class HornRegistration {
	private double scale;
	private int numpts;
	private Matrix centroidA, centroidB;
	private Matrix AM, BM, An, Bn, M, E, D, R, T;
	/**
	 * Class constructor that computes the rotation matrix and translation vector
	 * upon initialization.
	 * @param A the source matrix
	 * @param B the target matrix
	 * @param doScale whether to find the scaling factor
	 */
	public HornRegistration(Matrix A, Matrix B, boolean doScale) {
		this.AM = A;
		this.BM = B;
		this.numpts = A.getArray()[0].length;
		if (numpts < 4) {
			System.err.println("Need at least four point pairs.");
			System.exit(1);
		}
		this.getCentroids(AM,BM);
		this.subCentroids();
		this.M = this.computeQuaternion();
		this.E = M.eig().getV();
		this.D = M.eig().getD();
		this.R = this.computeRotation();
		if (doScale) {
			this.scale = this.scaleFactor();
		} else {
			this.scale = 1;
		}
		this.T = this.computeTranslation();
	}
	/**
	 * Calculates the centroids of matrices A and B
	 * @param A the matrix A
	 * @param B the matrix B
	 */
	private void getCentroids(Matrix A, Matrix B) {
		centroidA = mean(A,2);
		centroidB = mean(B,2);
	}
	/**
	 * Subtracts the centroids from the AM and BM matrices.
	 */
	private void subCentroids() {
		Matrix centroidATiled = this.repmat(centroidA, 1, numpts);
		Matrix centroidBTiled = this.repmat(centroidB, 1, numpts);
		this.An = this.AM.minus(centroidATiled);
		this.Bn = this.BM.minus(centroidBTiled);
	}
	public double scaleFactor() {
		double a = 0, b = 0;
		double s;
		for (int i = 0; i < this.numpts; i++) {
			a += this.Bn.getMatrix(0, this.Bn.getRowDimension() - 1, i, i).transpose().times(
					this.rotationMatrix().times(
							this.An.getMatrix(0, this.An.getRowDimension() - 1, i, i))).getArray()[0][0];
			b += this.Bn.getMatrix(0, this.Bn.getRowDimension() - 1, i, i).transpose().times(
					Bn.getMatrix(0, this.Bn.getRowDimension() - 1, i, i)).getArray()[0][0];
		}
		s = b/a;
		return s;
	}
	
	/**
	 * Computes the sum of the norms of the error
	 * @return
	 */
	public double error() {
		double error = 0;
		for (int i = 0; i < this.numpts; i++) {
			Matrix Bcol = this.BM.getMatrix(0, BM.getRowDimension() - 1, i, i);
			Matrix Acol = this.AM.getMatrix(0, AM.getRowDimension() - 1, i, i);
			Matrix T = this.transMatrix();
			Matrix e = Bcol.minus(this.rotationMatrix().times(Acol).times(this.scale).plus(T));
			error += e.normF();
		}
		return error;
	}
	/**
	 * Computes the quaternion to be used to compute the rotation matrix
	 * @return
	 */
	private Matrix computeQuaternion() {
		Matrix M = this.An.times(Bn.transpose());
		double Sxx = M.get(0,0), Sxy = M.get(0,1), Sxz = M.get(0,2),
				Syx = M.get(1,0), Syy = M.get(1,1), Syz = M.get(1,2),
				Szx = M.get(2,0), Szy = M.get(2,1), Szz = M.get(2,2);
		double[][] Narray = {{Sxx+Syy+Szz,Syz-Szy,Szx-Sxz,Sxy-Syx},
							 {Syz-Szy,Sxx-Syy-Szz,Sxy+Syx,Szx+Sxz},
							 {Szx-Sxz,Sxy+Syx,-Sxx+Syy-Szz,Syz+Szy},
							 {Sxy-Syx,Szx+Sxz,Syz+Szy,-Sxx-Syy+Szz}};
		Matrix N = new Matrix(Narray);
		return N;
	}
	/**
	 * Computes the rotation matrix
	 * @return the rotation matrix
	 */
	private Matrix computeRotation() {
		int maxindex = 0;
		for (int i = 0; i < D.getRowDimension(); i++) {
			if (this.D.get(i, i) > this.D.get(maxindex, maxindex)) {
				maxindex = i;
			}
		}
		Matrix maxE = this.E.getMatrix(0, E.getRowDimension() - 1, maxindex, maxindex);
		double scale = 1/maxE.normF();
		maxE = maxE.times(scale);
		double[] e = maxE.transpose().getArray()[0];
		double q0 = e[0], qx = e[1], qy = e[2], qz = e[3];
		double[][] Zarray = {{q0, -qz, qy},{qz, q0, -qx},{-qy, qx, q0}};
		Matrix v = maxE.getMatrix(1, 3, 0, 0);
		Matrix Z = new Matrix(Zarray);
		Matrix R = v.times(v.transpose()).plus(Z.times(Z));
		return R;
	}
	/**
	 * Computes the translation vector
	 * @return the translation vector
	 */
	private Matrix computeTranslation() {
		Matrix T = this.centroidB.minus(this.R.times(this.centroidA).times(this.scale));
		return T;
	}
	/**
	 * MATLAB's mean(A) implementation that gets the mean of a matrix's rows or column,
	 * depending on specification
	 * @param M the matrix
	 * @param dim the specified dimension, 1 for column and 2 for row
	 * @return the mean of either the columns or the rows
	 */
	private Matrix mean(Matrix M, int dim) {
		double[][] Ma = M.getArray();
		switch (dim) {
		case 1:
			double[] colMean = new double[Ma[0].length];
			for (int i = 0; i < Ma[0].length; i++) {
				double colSum = 0;
				for (int j = 0; j < Ma.length; j++) {
					colSum += Ma[j][i];
				}
				colMean[i] = colSum/Ma.length;
			}
			double[][] A = {colMean};
			return(new Matrix(A));
		case 2:
			double[] rowMean = new double[Ma.length];
			for (int i = 0; i < Ma.length; i++) {
				double rowSum = 0;
				for (int j = 0; j < Ma[0].length; j++) {
					rowSum += Ma[i][j];
				}
				rowMean[i] = rowSum/Ma[0].length;
			}
			double[][] B = new double[rowMean.length][1];
			for (int i = 0; i < rowMean.length; i++) {
				B[i][0] = rowMean[i];
			}
			return(new Matrix(B));
		default:
			System.err.println("Invalid dimension input.");
			System.exit(1);
			break;
		}
		return M;
	}
	/**
	 * MATLAB's repmat function that tiles a matrix by a specified number
	 * of row and column repetitions.
	 * @param M the matrix to be tiled
	 * @param reprows the number of row repetitions
	 * @param repcols the number of column repetitions
	 * @return
	 */
	private Matrix repmat(Matrix M, int reprows, int repcols) {
		int numrows = M.getArray().length * reprows;
		int numcols = M.getArray()[0].length * repcols;
		double[][] Ma = new double[numrows][numcols];
		double[][] singleRow = new double[M.getArray().length][repcols];
		for (int i = 0; i < M.getArray().length; i++)  {
			for (int j = 0; j < repcols; j++) {
				for (int k = 0; k < M.getArray()[0].length; k++) {
					singleRow[i][M.getArray()[0].length*j + k] = M.getArray()[i][k];
				}
			}
		}
		for (int i = 0; i < reprows; i++)  {
			for (int j = 0; j < M.getArray().length; j++) {
				Ma[M.getArray().length*i + j] = singleRow[j];
			}
		}
		Matrix repM = new Matrix(Ma);
		return repM;
	}
	/**
	 * Rotation matrix to be solved for.
	 * @return the solved rotation matrix
	 */
	public Matrix rotationMatrix() {
		return this.R;
	}
	/**
	 * Translation vector to be solved for.
	 * @return the solved translation vector
	 */
	public Matrix transMatrix() {
		return this.T;
	}
	public ColumnVector transVector() {
		return new ColumnVector(this.T);
	}
	public Matrix transAllPts() {
		return this.repmat(T, 1, numpts);
	}
	
	public Frame getFrame() {
		return new Frame(this.rotationMatrix(), this.transVector());
	}
}