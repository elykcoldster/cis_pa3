package cis.pa3.data;

import java.util.List;

import cis.pa3.tools.ColumnVector;
import cis.pa3.tools.PointCloud;

/**
 * An object that stores information from a rigid body data file
 * @author John Lee, Kyle Xiong
 *
 */
public class BodyData extends AbstractDataFileStorage {
	private PointCloud bodyMarkers;
	private ColumnVector tipCoordinates; 
	/**
	 * Returns the point cloud of marker coordinates
	 * @return the marker coordinates
	 */
	public PointCloud getMarkers() {
		return this.bodyMarkers;
	}
	/**
	 * Sets the body marker coordinates, given a point cloud
	 * @param markers the point cloud to set
	 */
	public void setMarkers(PointCloud markers) {
		this.bodyMarkers = markers;
	}	
	/**
	 * Gets the coordinates of the body tip
	 * @return the body tip coordinates
	 */
	public ColumnVector getTipCoordinates() {
		return tipCoordinates;
	}
	/**
	 * Sets the tip coordinates, given a ColumnVector object
	 * @param tipCoordinates the ColumnVector to set
	 */
	public void setTipCoordinates(ColumnVector tipCoordinates) {
		this.tipCoordinates = tipCoordinates;
	}
	/**
	 * Constructor for the body data storage object.
	 */
	public BodyData() {
		this.bodyMarkers = new PointCloud(3);
	}
	/**
	 * Converts a list of ColumnVector objects to a single PointCloud object.
	 */
	@Override
    public void columnVectorsToPointCloud(List<ColumnVector> vectors, List<Integer> parameters) {
		checkParameterSize(1, parameters, "BODY");
		this.parameters = parameters;
		int NMarkers = parameters.get(0);
		for (int i = 0; i < NMarkers; i++) {
			this.bodyMarkers.addPoint(vectors.get(i));
		}
		this.tipCoordinates = vectors.get(NMarkers);
		
    }

}
