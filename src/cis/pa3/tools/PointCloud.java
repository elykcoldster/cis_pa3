package cis.pa3.tools;

import java.util.ArrayList;
import java.util.List;

import Jama.Matrix;

/**
 * A data structure to store a set of points within a common frame
 * 
 * @author John Lee, Kyle Xiong
 *
 */

public class PointCloud {
	private final int dimension;
	private List<ColumnVector> points;
	
	public PointCloud(int dimension) {
		this.points = new ArrayList<>();
		this.dimension = dimension;
	}
	
	/**
	 * A constructor for turning a list of column vectors into 
	 * a point cloud
	 * @param points
	 */
	public PointCloud(List<ColumnVector> points) {
		this.dimension = points.get(0).getDimension();
		this.points = new ArrayList<>(points);
	}

	/**
	 * Adds a point defined by its coordinates to the point cloud;	 
	 * assumes that the new vector being added is in the same frame
	 * as the point cloud
	 * @param coordinates
	 *            the coordinates of the point to add
	 */
	public void addPoint(double... coordinates) {
		ColumnVector newVector = new ColumnVector(coordinates);
		checkDimensions(newVector.getDimension());
		checkUnique(newVector);
		this.points.add(newVector);
	}
	
	/**
	 * Adds a point defined as a column vector to the point cloud;
	 * assumes that the new vector being added is in the same frame
	 * as the point cloud
	 * @param newVector The column vector to add
	 */
	public void addPoint(ColumnVector newVector) {
		checkDimensions(newVector.getDimension());
		checkUnique(newVector);
		this.points.add(newVector);
	}

	/**
	 * Removes a point defined by its coordinates from the point cloud
	 * 
	 * @param coordinates
	 * @return
	 */
	public boolean removePoint(double... coordinates) {
		return this.points.remove(new ColumnVector(coordinates));
	}

	/**
	 * Applies a transformation matrix to all the points in the cloud
	 * 
	 * @param a
	 *            The transformation matrix with size m*n; n must equal
	 *            this.dimension
	 * 
	 */
	public void transform(Matrix a) {
		checkDimensions(a.getColumnDimension());
		List<ColumnVector> oldPoints = this.points;
		this.points = new ArrayList<>();
		for (ColumnVector point : oldPoints) {
			this.points.add(new ColumnVector(a.times(point.getMatrix())));
		}
	}

	/**
	 * Rotates a set of 3D points about a rotationAxis by theta radians
	 * 
	 * @param theta
	 *            The amount of rotation in radians
	 * @param rotationAxis
	 *            The 3D vector about which to rotate
	 */
	public void rotate3d(double theta, ColumnVector rotationAxis) {
		checkDimensions(rotationAxis.getDimension());
		if (this.dimension != 3) {
			throw new IllegalStateException("Must be 3D to rotate");
		}
		ColumnVector u = rotationAxis.unitVector();
		Matrix R = Matrix.identity(3, 3).times(Math.cos(theta));
		R = R.plus(u.skew().times(Math.sin(theta)));
		R = R.plus(u.outerProduct(u).times(1 - Math.cos(theta)));
		
		this.transform(R);
	}

	/**
	 * Applies a translation to all the points in the cloud
	 * 
	 * @param p
	 *            The translation vector; must be same dimensions as
	 *            this.dimesion
	 */
	public void translate(ColumnVector p) {
		checkDimensions(p.getDimension());
		List<ColumnVector> oldPoints = this.points;
		this.points = new ArrayList<>();
		for (ColumnVector point : oldPoints) {
			this.points.add(point.plus(p));
		}
	}

	/**
	 * Puts all the points in the cloud into the given frame
	 * 
	 * @param f
	 */
	public void frameShift(Frame f) {
		Matrix R = f.getRotationMatrix();
		ColumnVector p = f.getTranslationVector();
		this.transform(R);
		this.translate(p);
	}
	
	/**
	 * Returns the list of column vectors as a Matrix object of the form:
	 * x1, x2, ..., xn
	 * y1, y2, ..., yn
	 * z1, z2, ..., zn
	 * ...
	 * @return 
	 */
	public Matrix asMatrix() {
		double[][] matrixArray = new double[dimension][this.getSize()];
		for (int j = 0; j < this.getSize(); j++) {
			ColumnVector vectorArray = this.get(j);
			for (int i = 0; i < this.dimension; i++) {
				matrixArray[i][j] = vectorArray.get(i);
			}
		}
		return new Matrix(matrixArray);
	}
	
	/**
	 * Calculates the registration from this point cloud to another
	 * @param cloud The cloud to register to
	 * @return The registration object containing the transformation
	 */
	public HornRegistration register(PointCloud cloud) {
		return new HornRegistration(this.asMatrix(), cloud.asMatrix(), true);
	}

	/**
	 * Gets the i-th point in the point cloud
	 * 
	 * @param index
	 * @return
	 */
	public ColumnVector get(int index) {
		return this.points.get(index);
	}

	/**
	 * Gets the index of a given vector
	 * 
	 * @param point
	 *            The point to find
	 * @return The index of the point if found, else -1
	 */
	public int getIndexOf(ColumnVector point) {
		return this.points.indexOf(point);
	}

	/**
	 * Gets the number of points in the point cloud
	 * @return
	 */
	public int getSize() {
		return this.points.size();
	}
	
	public int getDimension() {
		return this.dimension;
	}

	/**
	 * Returns a deep copy of this point cloud
	 * 
	 * @return The cloned PointCloud
	 */
	public PointCloud clone() {
		PointCloud clone = new PointCloud(this.dimension);
		List<ColumnVector> points = new ArrayList<>(this.points);
		clone.points = points;
		return clone;
	}
	
	/**
	 * Takes inner products between two point clouds
	 * If they are different sizes, the first n are inner products are calculated,
	 * where n = min(this.size, cloud.size), and the rest are filled with the rest
	 * of the values in the larger point cloud
	 * @param cloud The PointCloud with which to take the inner product
	 * @return
	 */
	public double[] innerProducts(PointCloud cloud) {
		checkDimensions(cloud.getDimension());
		int size = Math.max(cloud.getSize(), this.getSize());
		double[] values = new double[size];
		for (int i = 0; i < size; i++) {
			values[i] = this.get(i).innerProduct(cloud.get(i));
		}
		return values;
	}
	public static Matrix colTiledCentroid(ColumnVector cv, int numcols) {
		Matrix col = cv.getMatrix();
		Matrix M = new Matrix(col.getRowDimension(), numcols);
		for (int i = 0; i < numcols; i++) {
			M.setMatrix(0, col.getRowDimension() - 1, i, i, col);
		}
		return M;
	}
	public static ColumnVector centroid(PointCloud pc) {
		Matrix M = pc.asMatrix();
		Matrix col = new Matrix(M.getRowDimension(), 1);
		for(int i = 0; i < M.getColumnDimension(); i++) {
			col = col.plus(M.getMatrix(0, M.getRowDimension() - 1, i, i));
		}
		col = col.times(1.0/M.getColumnDimension());
		return (new ColumnVector(col));
	}
	public static List<ColumnVector> centroidList(List<PointCloud> lpc) {
		List<ColumnVector> collist = new ArrayList<>();
		for (int i = 0; i < lpc.size(); i++) {
			collist.add(centroid(lpc.get(i)));
		}
		return collist;
	}
	public static PointCloud matrixToPointCloud(Matrix mat) {
		PointCloud cloud = new PointCloud(mat.getRowDimension());
		for (int i = 0; i < mat.getColumnDimension(); i++) {
			cloud.addPoint(new ColumnVector(mat.getMatrix(0, mat.getRowDimension() - 1, i, i)));
		}
		return cloud;
	}
	
	public static List<PointCloud> matrixToPointCloudList(Matrix mat, int numFrames) {
		List<PointCloud> pointClouds = new ArrayList<>();
		int pointsPerFrame = mat.getColumnDimension() / numFrames;
		for (int i = 0; i < numFrames; i++) {
			pointClouds.add(PointCloud.matrixToPointCloud(mat.getMatrix(
					0, mat.getRowDimension() - 1, i*pointsPerFrame, (i+1)*pointsPerFrame - 1)));
		}
		return pointClouds;
	}
	
	public static Matrix pointCloudListToMatrix(List<PointCloud> clouds) {
		int numframes = clouds.size();
		int numpoints = clouds.get(0).asMatrix().getColumnDimension();
		Matrix matrixClouds = new Matrix(3, numpoints * numframes);
		for (int i = 0; i < clouds.size(); i++) {
			matrixClouds.setMatrix(0, 2, i*numpoints, i*numpoints + numpoints - 1, clouds.get(i).asMatrix());
		}
		return matrixClouds;
	}

	private void checkDimensions(int dimension) {
		if (this.dimension != dimension) {
			throw new IllegalArgumentException(
			        "Dimension mismatch; vectors are of size " + this.dimension);
		}
	}

	private void checkUnique(ColumnVector point) {
		if (this.points.contains(point)) {
			throw new IllegalArgumentException("Point already exists in point cloud");
		}
	}


}
