package cis.pa3.geometry;

import java.util.ArrayList;
import java.util.List;

import cis.pa3.tools.ColumnVector;
import cis.pa3.tools.Math3D;
import cis.pa3.tools.Mesh;

public class Utilities {
	private static final int DIM = 3;
	/**
	 * Finds the closest point to a source point on the triangle
	 * @param tri the triangle to search in
	 * @param source the source point
	 * @return the closest point
	 */
	public static ColumnVector findClosestPointOnTriangle(Triangle tri, ColumnVector source) {
		ColumnVector closestPoint = new ColumnVector(DIM);
		ColumnVector edge1 = tri.getVertex(1).minus(tri.getVertex(0));
		ColumnVector edge2 = tri.getVertex(2).minus(tri.getVertex(0));
		ColumnVector baseToSource = tri.getVertex(0).minus(source);
		
		double a = Math3D.dotProduct(edge1, edge1);
		double b = Math3D.dotProduct(edge2, edge1);
		double c = Math3D.dotProduct(edge2, edge2);
		double d = Math3D.dotProduct(edge1, baseToSource);
		double e = Math3D.dotProduct(edge2, baseToSource);
		// gradient of Q = as^2 + 2bst + ct^2 + 2ds + 2et + f
		// is zero when s and t are zero:
		double det = a*c - b*b;
		double s = b*e - c*d;
		double t = b*d - a*e;
		
		if (s + t <= det) {
			if (s < 0) {
				if (t < 0) {
					if (d < 0) {
						s = Math.max(0, Math.min(-d/a, 1));
						t = 0;
					} else {
						s = 0;
						t = Math.max(0, Math.min(-e/c, 1));
					}
				} else {
					s = 0;
					t = Math.max(0, Math.min(-e/c, 1));
				}
			} else if (t < 0) {
				s = Math.max(0, Math.min(-d/a, 1));
				t = 0;
			} else {
				double invdet = 1.0/det;
				s = s*invdet;
				t = t*invdet;
			}
		} else {
			if (s < 0) {
				if (c + e > b + d) {
					double num = c + e - (b + d);
					double denom = a - 2*b + c;
					s = Math.max(0, Math.min(num/denom, 1));
					t = 1 - s;
				} else {
					s = 0;
					t = Math.max(0, Math.min(-e/c, 1));
				}
			} else if (t < 0) {
				if (a + d < b + e) {
					double num = c + e - (b + d);
					double denom = a - 2*b + c;
					s = Math.max(0, Math.min(num/denom, 1));
					t = 1 - s;
				} else {
					s = Math.max(0, Math.min(-e/c, 1));
					t = 0;
				}
			} else {
				double num = c + e - (b + d);
				double denom = a - 2*b + c;
				s = Math.max(0, Math.min(num/denom, 1));
				t = 1 - s;
			}
		}
		closestPoint = tri.getVertex(0).plus(edge1.times(s)).plus(edge2.times(t));
		return closestPoint;
		}
	/**
	 * Finds the closest point to a source point on the mesh.
	 * @param mesh the mesh to search in
	 * @param source the source point
	 * @return the closest point on the mesh
	 */
	public static ColumnVector findClosestPointOnMesh(Mesh mesh, ColumnVector source) {
		List<ColumnVector> closestPoints = new ArrayList<>();
		for (int i = 0; i < mesh.getTriangles().size(); i++){
			closestPoints.add(findClosestPointOnTriangle(mesh.getTriangles().get(i), source));
		}
		ColumnVector min = source.minus(closestPoints.get(0));
		for (int i = 0; i < closestPoints.size(); i++) {
			ColumnVector temp = source.minus(closestPoints.get(i));
			if (temp.magnitude() < min.magnitude()) {
				min = temp;
			}
		}
		return min;
	}
}
