package cis.pa3.data;

import java.util.List;

import cis.pa3.tools.ColumnVector;
import cis.pa3.tools.PointCloud;

/**
 * An object that stores information from a mesh data file
 * @author John Lee, Kyle Xiong
 *
 */
public class MeshData extends AbstractDataFileStorage {
	private PointCloud vertices;
	private PointCloud triangles;
	/**
	 * Get the vertex coordinates
	 * @return the vertex coordinates
	 */
	public PointCloud getVertices() {
		return this.vertices;
	}
	/**
	 * Get the vertex indices of the triangles
	 * @return the vertex indices
	 */
	public PointCloud getTriangles() {
		return this.triangles;
	}
	/**
	 * Set the vertices in the mesh, given a point cloud
	 * @param verts the point cloud to set
	 */
	public void setVertices(PointCloud verts) {
		this.vertices = verts;
	}
	/**
	 * Set the point cloud of triangle vertex indices
	 * @param tris the point cloud to set
	 */
	public void setTriangles(PointCloud tris) {
		this.triangles = tris;
	}
	/**
	 * Constructor for the mesh data storage object
	 */
	public MeshData() {
		this.vertices = new PointCloud(3);
		this.triangles = new PointCloud(3);
	}
	/**
	 * Converts a list of ColumnVector objects to a single PointCloud object.
	 */
	@Override
    public void columnVectorsToPointCloud(List<ColumnVector> vectors, List<Integer> parameters) {
		checkParameterSize(2, parameters, "MESH");
		this.parameters = parameters;
		int NVertices = parameters.get(0);
		int NTriangles = parameters.get(1);
		for (int i = 0; i < NVertices; i++) {
			this.vertices.addPoint(vectors.get(i));
		}
		for (int i = 0; i < NTriangles; i++) {
			ColumnVector longVector = vectors.get(i + NVertices);
			double[] shortVectorArray = {longVector.get(0), longVector.get(1), longVector.get(2)};
			ColumnVector shortVector = new ColumnVector(shortVectorArray);
			this.triangles.addPoint(shortVector);
		}
    }

}
