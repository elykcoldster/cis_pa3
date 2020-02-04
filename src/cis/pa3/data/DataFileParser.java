package cis.pa3.data;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import cis.pa3.tools.ColumnVector;

/**
 * A utility class that parses each of the different kinds of data files.
 * Designed Specifically for PA3 and PA4
 * @author John Lee, Kyle Xiong
 *
 */
public class DataFileParser {
	private static final int DIM = 3;
	private List<Integer> readParameters(BufferedReader br, String delimiter) {
		String line;
		List<Integer> params = new ArrayList<>();
		try {
			// Default case is reading a body file where 'line' gets two elements.
			line = br.readLine();
			String[] input = line.split(delimiter);
			for (int i = 0; i < input.length; i++) {
				if (isInteger(input[i])) {
					params.add(Integer.parseInt(input[i]));
				}
			}
			if (input.length == 1) {	// this is true for mesh files
				int vnum = Integer.parseInt(line);
				for (int i = 0; i < vnum; i++) {
					line = br.readLine();
				}
				line = br.readLine();
				params.add(Integer.parseInt(line));
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
		return params;
		
	}
	
	private List<ColumnVector> convertToColumnVectors(List<List<Double>> coordinateList) {
		List<ColumnVector> columnVectorList = new ArrayList<>();
		for (int i = 0; i < coordinateList.size(); i++) {
			int size = coordinateList.get(i).size();
			double[] vecarray = new double[size];
			for (int j = 0; j < vecarray.length; j++) {
				vecarray[j] = coordinateList.get(i).get(j);
			}
			columnVectorList.add(new ColumnVector(vecarray));
		}		
		return columnVectorList;
	}
	
	private List<List<Double>> readPoints(BufferedReader br, String delimiter) {
		
		List<List<Double>> coordinateList = new ArrayList<>();
		String line;
		try {
			br.readLine();
			while ((line = br.readLine()) != null) {
				String[] input = line.split(delimiter);
				if (input.length >= DIM) {
					ArrayList<Double> tempcoords = new ArrayList<Double>();
					for (int i = 0; i < input.length; i++) {
						if (isDouble(input[i])) {
							tempcoords.add(Double.parseDouble(input[i]));
						}
					}
					coordinateList.add(tempcoords);
				}
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
		return coordinateList;
		
	}
	private void parseDataFile(DataFileStorage data, String path, String delimiter) {
		try {
			// separate buffered readers due to the nature of the input files 
			BufferedReader paramreader = new BufferedReader(new FileReader(path));
			BufferedReader pointreader = new BufferedReader(new FileReader(path));
			List<Integer> parameters = this.readParameters(paramreader, delimiter);
			List<ColumnVector> vectors = this.convertToColumnVectors(this.readPoints(pointreader, delimiter));
			data.columnVectorsToPointCloud(vectors, parameters);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	public BodyData parseBodyData(String path) {
		BodyData data = new BodyData();
		this.parseDataFile(data, path, "\\s+");
		return data;
	}
	public MeshData parseMeshData(String path) {
		MeshData data = new MeshData();
		this.parseDataFile(data, path, "\\s+");
		return data;
	}
	
	public SampleReadingsData parseSampleReadingsData(String readingsPath, String bodyAPath, String bodyBPath) {
		BodyData bodyDataA = this.parseBodyData(bodyAPath);
		BodyData bodyDataB = this.parseBodyData(bodyBPath);
		SampleReadingsData readingsData = new SampleReadingsData(bodyDataA, bodyDataB);
		this.parseDataFile(readingsData, readingsPath, "[, \t]");
		return readingsData;
	}
	
	private boolean isInteger(String str) {
		try {Integer.parseInt(str);} catch(NumberFormatException e) {return false;}
		return true;
	}
	/**
	 * Checks if a string input is a double
	 * @param str string input
	 * @return if string input is a double
	 */
	private boolean isDouble(String str) {
		try {Double.parseDouble(str);} catch(NumberFormatException e) {return false;}
		return true;
	} 

	public Output3Data parseOutput3Data(String path) {
		Output3Data data = new Output3Data();
		this.parseDataFile(data, path, "\\s+");
		return data;
	}

}
