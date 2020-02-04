package cis.pa3.tests;

import java.util.ArrayList;
import java.util.List;

import cis.pa3.geometry.Triangle;
import cis.pa3.geometry.Utilities;
import cis.pa3.tools.ColumnVector;

public class TestFCP {
	public static void main(String[] args) {
		// make an arbitrary triangle to test FCPoT function
		double[] v1coords = {0.0,0.0,0.0};
		double[] v2coords = {2.0,2.0,0.0};
		double[] v3coords = {4.0,0,0.0};
		// project onto right side of triangle
		double[] source1 = {0.0, 1.0, 2.0};
		double[] source2 = {2.0, 4.0, 2.0};
		double[] source3 = {4.0, 1.0, 2.0};
		double[] source4 = {5.0, -0.5, 2.0};
		double[] source5 = {2.0, -1.0, 2.0};
		double[] source6 = {-2.0, -1.0, 2.0};
		double[] source0 = {2.0, 1.0, 2.0};
		List<ColumnVector> verts = new ArrayList<>();
		verts.add(new ColumnVector(v1coords));
		verts.add(new ColumnVector(v2coords));
		verts.add(new ColumnVector(v3coords));
		ColumnVector sourcevec1 = new ColumnVector(source1);
		ColumnVector sourcevec2 = new ColumnVector(source2);
		ColumnVector sourcevec3 = new ColumnVector(source3);
		ColumnVector sourcevec4 = new ColumnVector(source4);
		ColumnVector sourcevec5 = new ColumnVector(source5);
		ColumnVector sourcevec6 = new ColumnVector(source6);
		ColumnVector sourcevec0 = new ColumnVector(source0);
		Triangle testtri = new Triangle(verts);
		ColumnVector closestpoint = Utilities.findClosestPointOnTriangle(testtri, sourcevec1);
		closestpoint.getMatrix().print(0, 2);
		closestpoint = Utilities.findClosestPointOnTriangle(testtri, sourcevec2);
		closestpoint.getMatrix().print(0, 2);
		closestpoint = Utilities.findClosestPointOnTriangle(testtri, sourcevec3);
		closestpoint.getMatrix().print(0, 2);
		closestpoint = Utilities.findClosestPointOnTriangle(testtri, sourcevec4);
		closestpoint.getMatrix().print(0, 2);
		closestpoint = Utilities.findClosestPointOnTriangle(testtri, sourcevec5);
		closestpoint.getMatrix().print(0, 2);
		closestpoint = Utilities.findClosestPointOnTriangle(testtri, sourcevec6);
		closestpoint.getMatrix().print(0, 2);
		closestpoint = Utilities.findClosestPointOnTriangle(testtri, sourcevec0);
		closestpoint.getMatrix().print(0, 2);
	}
}