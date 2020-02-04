package cis.pa3.drivers;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;

import Jama.Matrix;
import cis.pa3.data.BodyData;
import cis.pa3.data.DataFileParser;
import cis.pa3.data.MeshData;
import cis.pa3.data.Output3Data;
import cis.pa3.data.SampleReadingsData;
import cis.pa3.geometry.Utilities;
import cis.pa3.tools.ColumnVector;
import cis.pa3.tools.Frame;
import cis.pa3.tools.HornRegistration;
import cis.pa3.tools.MatrixHelper;
import cis.pa3.tools.Mesh;
import cis.pa3.tools.PointCloud;

public class MatchingDriver {
	private static final int DIM = 3;
	private BodyData bodyA, bodyB;
	private SampleReadingsData sampleReadings;
	private Mesh mesh;
	private List<ColumnVector> DkVectors, SkVectors, CkVectors;
	private Frame Freg;
	private char letter;
	private String datasetType;
	
	public MatchingDriver(char letter, boolean isDebug) {
		this.letter = letter;
		DataFileParser parser = new DataFileParser();
		String bodyAPath = "data" + File.separator + "Problem3-BodyA.txt";
		String bodyBPath = "data" + File.separator + "Problem3-BodyB.txt";
		String meshpath = "data" + File.separator + "Problem3Mesh.sur";
		String sampleReadingsPath;
		if (isDebug) {
			this.datasetType = "Debug";
		}
		else {
			this.datasetType = "Unknown";
		}
		sampleReadingsPath = "data" + File.separator + "PA3-" + letter + "-" + this.datasetType + "-SampleReadingsTest.txt";

		this.bodyA = parser.parseBodyData(bodyAPath);
		this.bodyB = parser.parseBodyData(bodyBPath);
		this.sampleReadings = parser.parseSampleReadingsData(sampleReadingsPath, bodyAPath, bodyBPath);
		MeshData meshA = parser.parseMeshData(meshpath);
		this.mesh = new Mesh(meshA);
		this.Freg = new Frame(Matrix.identity(DIM, DIM), new ColumnVector(DIM));
		this.calcDkVectors();
		this.calcSkVectors();
		this.calcCkVectors();
	}
	
	private void calcDkVectors() {	
		PointCloud bodyACloud = this.bodyA.getMarkers();
		PointCloud bodyBCloud = this.bodyB.getMarkers();
		ColumnVector Atip = this.bodyA.getTipCoordinates();
		this.DkVectors = new ArrayList<>();

		for (int i = 0; i < sampleReadings.getNsamps(); i++) {
			PointCloud trackerACloud = this.sampleReadings.getAMarkerCloud().get(i);
			PointCloud trackerBCloud = this.sampleReadings.getBMarkerCloud().get(i);
			HornRegistration Areg = new HornRegistration(bodyACloud.asMatrix(), trackerACloud.asMatrix(), true);
			HornRegistration Breg = new HornRegistration(bodyBCloud.asMatrix(), trackerBCloud.asMatrix(), true);
			Frame FAk = Areg.getFrame();
			Frame FBk = Breg.getFrame();			
			this.DkVectors.add(FBk.inverse().combineFrames(FAk).timesVector(Atip));
		}
	}
	
	private void calcSkVectors() {
		this.SkVectors = new ArrayList<>();
		for (int i = 0; i < this.DkVectors.size(); i++) {
			ColumnVector trans = this.Freg.timesVector(this.DkVectors.get(i));
			this.SkVectors.add(trans);
		}
	}
	private void calcCkVectors() {
		this.CkVectors = new ArrayList<>();
		for (int i = 0; i < this.SkVectors.size(); i++) {
			ColumnVector closest = Utilities.findClosestPointOnMesh(this.mesh, this.SkVectors.get(i));
			this.CkVectors.add(this.SkVectors.get(i).minus(closest));
		}
	}
	public List<ColumnVector> getDkVectors() {
		return this.DkVectors;
	}
	public List<ColumnVector> getSkVectors() {
		return this.SkVectors;
	}
	public List<ColumnVector> getCkVectors() {
		return this.CkVectors;
	}
	

	public List<Double> getSkCkDiff() {
		List<Double> error = new ArrayList<>();
		for (int i = 0; i < this.SkVectors.size(); i++) {
			ColumnVector diff = this.SkVectors.get(i).minus(this.CkVectors.get(i));
			error.add(diff.magnitude());
		}		
		return error;
	}
	
	/**
	 * Compares to the expected output for the debug datasets to test whether our output is as expected 
	 */
	public void compareToOutput() {
		DataFileParser parser = new DataFileParser();
		String path = "data" + File.separator + "PA3-" + this.letter + "-Debug-Output.txt";
		Output3Data outputData = parser.parseOutput3Data(path);
		
		System.out.println("Differences in output for dataset " + letter);

		Matrix dkDifference = compareDkToOutput(outputData).transpose();		
		Matrix ckDifference = compareCkToOutput(outputData).transpose();
		Matrix errorDifference = compareErrorsToOutput(outputData).transpose();
		Matrix outputMatrix = MatrixHelper.stackHorizontally(dkDifference, ckDifference, errorDifference);
		outputMatrix.print(8, 3);
		
		double dkNorm = dkDifference.normF() / dkDifference.getRowDimension();
		double ckNorm = ckDifference.normF() / ckDifference.getRowDimension();
		double errorNorm = errorDifference.normF() / errorDifference.getRowDimension();
		System.out.printf("Average norm of error per vector for dk, ck, and differences: %.3f\t%.3f\t%.3f\n", dkNorm, ckNorm, errorNorm);
	}
	
	private Matrix compareDkToOutput(Output3Data outputData) {
		PointCloud expectedDkVectors = new PointCloud(outputData.getdVectors());
		PointCloud actualDkVectors = new PointCloud(this.DkVectors);
		return expectedDkVectors.asMatrix().minus(actualDkVectors.asMatrix());
	}
	
	private Matrix compareCkToOutput(Output3Data outputData) {
		PointCloud expectedCkVectors = new PointCloud(outputData.getcVectors());
		PointCloud actualCkVectors = new PointCloud(this.CkVectors);
		return expectedCkVectors.asMatrix().minus(actualCkVectors.asMatrix());
	}
	
	private Matrix compareErrorsToOutput(Output3Data outputData) {
		Matrix actualErrors = MatrixHelper.convertListToMatrix(this.getSkCkDiff());
		Matrix expectedErrors = MatrixHelper.convertListToMatrix(outputData.getDifferenceMagnitudes());
		return expectedErrors.minus(actualErrors);
	}
	
	/**
	 * Creates the output for PA3
	 * @param datasetType
	 */
	public void createOutput() {
		try {
	        PrintWriter writer = new PrintWriter(
	        		"output" + File.separator + "PA3-" + this.letter + "-" + this.datasetType + "-Output.txt");
	        Matrix dkMatrix = new PointCloud(this.DkVectors).asMatrix().transpose();
			Matrix ckMatrix = new PointCloud(this.CkVectors).asMatrix().transpose();
			Matrix errorMatrix = MatrixHelper.convertListToMatrix(this.getSkCkDiff()).transpose();
			Matrix outputMatrix = MatrixHelper.stackHorizontally(dkMatrix, ckMatrix, errorMatrix);
			outputMatrix.print(writer, 8, 3);
			writer.close();
        } catch (FileNotFoundException e) {
	        e.printStackTrace();
        }		
	}
	
	public static void main(String[] args) {
		MatchingDriver driver;
		for (char letter = 'A'; letter <= 'F'; letter++) {
			driver = new MatchingDriver(letter, true);						
			driver.compareToOutput();
			driver.createOutput();
		}
		for (char letter = 'G'; letter <= 'J'; letter++) {
			if (letter == 'I') {
				continue;
			}
			driver = new MatchingDriver(letter, false);
			driver.createOutput();
			
		}
	}
}
