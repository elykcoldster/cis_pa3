package cis.pa3.data;

import java.util.List;

import cis.pa3.tools.ColumnVector;

/**
 * An interface that describes a data file data structure
 * @author John Lee, Kyle Xiong
 *
 */
public interface DataFileStorage {
	/**
	 * Converts a list of vectors and parameters into the appropriate data structure object 
	 * @param vectors The vectors to add
	 * @param parameters The parameters to base off of
	 */
	public void columnVectorsToPointCloud(List<ColumnVector> vectors, List<Integer> parameters);
}
