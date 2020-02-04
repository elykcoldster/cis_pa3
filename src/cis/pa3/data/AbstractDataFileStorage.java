package cis.pa3.data;

import java.util.List;

import cis.pa3.tools.ColumnVector;

/**
 * An abstract class that stores values from a data file
 * @author John Lee, Kyle Xiong
 *
 */
public abstract class AbstractDataFileStorage implements DataFileStorage {
	
	protected List<Integer> parameters;

	public List<Integer> getParameters() {
		return parameters;
	}

	public void setParameters(List<Integer> parameters) {
		this.parameters = parameters;
	}

	@Override
	public abstract void columnVectorsToPointCloud(List<ColumnVector> vectors, List<Integer> parameters);
	
	/**
	 * Checks the size of the parameter list
	 * @param size The size to check the parameter list with
	 * @param parameters The list of parameters
	 * @param name The type of data file
	 */
	protected void checkParameterSize(int size, List<Integer> parameters, String name) {
		if (size != parameters.size()) {
			throw new IllegalArgumentException(name + " takes in " + size + " parameters!");
		}
		
	}

}
