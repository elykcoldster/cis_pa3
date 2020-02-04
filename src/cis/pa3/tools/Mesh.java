package cis.pa3.tools;

import java.util.ArrayList;
import java.util.List;

import cis.pa3.data.MeshData;
import cis.pa3.geometry.Triangle;;

public class Mesh {
	private static final int DIM = 3;
	private PointCloud vertexdata;
	private PointCloud tridata;
	private List<Triangle> triangles;
	public Mesh(MeshData meshdata) {
		this.vertexdata = meshdata.getVertices();
		this.tridata = meshdata.getTriangles();
		this.triangles = new ArrayList<>();
		this.makeTriangles();
	}
	public void setVertices(PointCloud vertices) {
		this.vertexdata = vertices;
	}
	public void setTriangles(PointCloud triangles) {
		this.tridata = triangles;
	}
	public PointCloud getVertices() {
		return this.vertexdata;
	}
	public List<Triangle> getTriangles() {
		return this.triangles;
	}
	private void makeTriangles() {
		for (int i = 0; i < this.tridata.getSize(); i++) {
			ColumnVector vindices = this.tridata.get(i);
			List<ColumnVector> vertices = new ArrayList<>();
			for (int j = 0; j < DIM; j++) {
				vertices.add(this.vertexdata.get((int) vindices.get(j)));
			}
			this.triangles.add(new Triangle(vertices));
		}
	}
}