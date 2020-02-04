package cis.pa3.data;

import java.util.ArrayList;
import java.util.List;

import cis.pa3.tools.ColumnVector;

public class Output3Data extends AbstractDataFileStorage {
	private List<ColumnVector> dVectors, cVectors;
	private List<Double> differenceMagnitudes;
	/**
	 * Constructor for an object that stores data from the output files
	 */
	public Output3Data() {
		this.dVectors = new ArrayList<>();
		this.cVectors = new ArrayList<>();
		this.differenceMagnitudes = new ArrayList<>();
	}
	/**
	 * Get the d_k vectors in the output file
	 * @return the d_k vectors
	 */
	public List<ColumnVector> getdVectors() {
		return dVectors;
	}
	/**
	 * Set the d_k vectors as the parameter
	 * @param dVectors the d_k vectors to set
	 */
	public void setdVectors(List<ColumnVector> dVectors) {
		this.dVectors = dVectors;
	}
	/**
	 * Get the c_k vectors
	 * @return the c_k vectors
	 */
	public List<ColumnVector> getcVectors() {
		return cVectors;
	}
	/**
	 * Set the c_k vectors as the parameter
	 * @param cVectors the c_k vectors to set
	 */
	public void setcVectors(List<ColumnVector> cVectors) {
		this.cVectors = cVectors;
	}
	/**
	 * Get the difference in magnitudes
	 * @return difference in magnitudes
	 */
	public List<Double> getDifferenceMagnitudes() {
		return differenceMagnitudes;
	}
	/**
	 * Set the difference in magnitudes
	 * @param differenceMagnitudes the list of magnitudes to set
	 */
	public void setDifferenceMagnitudes(List<Double> differenceMagnitudes) {
		this.differenceMagnitudes = differenceMagnitudes;
	}
	/**
	 * Converts a list of ColumnVector objects to a single PointCloud object.
	 */
	@Override
    public void columnVectorsToPointCloud(List<ColumnVector> vectors, List<Integer> parameters) {
	    this.checkParameterSize(2, parameters, "pa3-Output");
	    int Nsamps = parameters.get(0);
	    if (Nsamps != vectors.size()) {
	    	throw new IllegalArgumentException("Input vectors must equal number of samples");
	    }
	    for (ColumnVector longVector : vectors) {
	    	ColumnVector dVector = new ColumnVector(longVector.getMatrix().getMatrix(0, 2, 0, 0));
	    	ColumnVector cVector = new ColumnVector(longVector.getMatrix().getMatrix(3, 5, 0, 0));
	    	double magnitude = longVector.get(6);
	    	dVectors.add(dVector);
	    	cVectors.add(cVector);
	    	differenceMagnitudes.add(magnitude);	    	
	    }
    }

}
