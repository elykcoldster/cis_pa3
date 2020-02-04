package cis.pa3.data;

import java.util.ArrayList;
import java.util.List;

import cis.pa3.tools.ColumnVector;
import cis.pa3.tools.PointCloud;

public class SampleReadingsData extends AbstractDataFileStorage{
	private List<PointCloud> AMarkerList, BMarkerList;
	private int NA, NB, Nsamps;
	/**
	 * Get the number of samples
	 * @return number of samples
	 */
	public int getNsamps() {
		return Nsamps;
	}
	/**
	 * Set the number of samples
	 * @param nsamps the number of samples to set
	 */
	public void setNsamps(int nsamps) {
		Nsamps = nsamps;
	}
	/**
	 * Get all of frames of the a_k vectors
	 * @return the list of a_k pointclouds
	 */
	public List<PointCloud> getAMarkerCloud() {
		return AMarkerList;
	}
	/**
	 * Set the A marker point cloud list
	 * @param aMarkerCloud the point cloud list to set
	 */
	public void setAMarkerCloud(List<PointCloud> aMarkerCloud) {
		AMarkerList = aMarkerCloud;
	}
	/**
	 * Get all of frames of the b_k vectors
	 * @return the list of b_k pointclouds
	 */
	public List<PointCloud> getBMarkerCloud() {
		return BMarkerList;
	}
	/**
	 * Set the B marker point cloud list
	 * @param aMarkerCloud the point cloud list to set
	 */
	public void setBMarkerCloud(List<PointCloud> bMarkerCloud) {
		BMarkerList = bMarkerCloud;
	}
	/**
	 * Get N_A
	 * @return N_A
	 */
	public int getNA() {
		return NA;
	}
	/**
	 * Set N_A
	 * @param nA N_A
	 */
	public void setNA(int nA) {
		NA = nA;
	}
	/**
	 * Get N_B
	 * @return N_B
	 */
	public int getNB() {
		return NB;
	}
	/**
	 * Set N_B
	 * @param nB N_B
	 */
	public void setNB(int nB) {
		NB = nB;
	}
	/**
	 * The constructor for an object that stores data from sample
	 * readings files
	 * @param bodyA the body data for body A
	 * @param bodyB the body data for body B
	 */
	public SampleReadingsData(BodyData bodyA, BodyData bodyB) {
		this.NA = bodyA.parameters.get(0);
		this.NB = bodyB.parameters.get(0);
    }
	/**
	 * Converts a list of ColumnVector objects to a single PointCloud object.
	 */
	@Override
    public void columnVectorsToPointCloud(List<ColumnVector> vectors, List<Integer> parameters) {
		checkParameterSize(3, parameters, "SampleReadings");
		this.parameters = parameters;
		int NS = this.parameters.get(0);
		this.Nsamps = this.parameters.get(1);
		
		this.AMarkerList = new ArrayList<>();
		this.BMarkerList = new ArrayList<>();
		
		for (int i = 0; i < Nsamps; i++) {
			PointCloud AMarkers = new PointCloud(3);
			PointCloud BMarkers = new PointCloud(3);
			int offset = i * NS;
			for (int j = 0; j < this.NA; j++) {
				AMarkers.addPoint(vectors.get(offset + j));
			}
			for (int j = NA; j < this.NA + this.NB; j++) {
				BMarkers.addPoint(vectors.get(offset + j));
			}
			this.AMarkerList.add(AMarkers);
			this.BMarkerList.add(BMarkers);
		}

    }

}
