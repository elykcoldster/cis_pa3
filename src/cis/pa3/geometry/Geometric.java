package cis.pa3.geometry;

import java.util.List;

import cis.pa3.tools.ColumnVector;

public interface Geometric {
	public List<ColumnVector> vertices();
	public ColumnVector getVertex(int index);
	public void setVertex(int index, ColumnVector vertex);
	public void setGeo(List<ColumnVector> v);
	public void checkValid();
}