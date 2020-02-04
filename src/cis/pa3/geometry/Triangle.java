package cis.pa3.geometry;

import java.util.ArrayList;
import java.util.List;

import cis.pa3.tools.ColumnVector;

public class Triangle implements Geometric{
	private static final int DIM = 3;
	private static final int VERTNUM = 3;
	private List<ColumnVector> vertices;
	public Triangle(List<ColumnVector> v) {
		this.vertices = v;
		this.checkValid();
	}
	public Triangle() {
		this.vertices = new ArrayList<ColumnVector>();
		for (int i = 0; i < VERTNUM; i++) {
			this.vertices.add(new ColumnVector(DIM)); // empty triangle
		}
		this.checkValid();
	}
	public List<ColumnVector> vertices() {
		return this.vertices;
	}
	public ColumnVector getVertex(int index) {
		return this.vertices.get(index);
	}
	public void setVertex(int index, ColumnVector edge) {
		this.vertices.set(index, edge);
		this.checkValid();
	}
	public void setGeo(List<ColumnVector> v) {
		this.vertices = v;
		this.checkValid();
	}
	public void checkValid() {
		if (this.vertices.size() != 3) {
			System.err.println("Invalid triangle dimensions.");
			System.exit(1);
		}
	}
	public String toString() {
		String s = "";
		for (int i = 0; i < this.vertices().size(); i++) {
			s += this.vertices.get(i).get(0) + " " + this.vertices.get(i).get(1) + " " + this.vertices.get(i).get(2) + "\n";
		}
		return s;
	}
}