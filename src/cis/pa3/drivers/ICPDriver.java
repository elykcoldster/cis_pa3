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
import cis.pa3.tools.Math3D;
import cis.pa3.tools.MatrixHelper;
import cis.pa3.tools.Mesh;
import cis.pa3.tools.PointCloud;

public class ICPDriver {
	private static final int DIM = 3;
	// maximum iteration number
	private static final int MAX_ITER = 1000;
	private static final double THRESHOLD_STRENGTH = 0.1, MIN_THRESHOLD = 0.01;
	private BodyData bodyA, bodyB;
	private SampleReadingsData sampleReadings;
	private Mesh mesh;
	private List<ColumnVector> DkVectors, SkVectors, CkVectors, errorvecs, A, B;
	private  List<Double> error;
	private Frame Freg;
	private char letter;
	private String datasetType;
	
	public ICPDriver(char letter, boolean isDebug) {
		this.letter = letter;
		DataFileParser parser = new DataFileParser();
		String bodyAPath = "data" + File.separator + "Problem4-BodyA.txt";
		String bodyBPath = "data" + File.separator + "Problem4-BodyB.txt";
		String meshpath = "data" + File.separator + "Problem3Mesh.sur";
		String sampleReadingsPath;
		if (isDebug) {
			this.datasetType = "Debug";
		}
		else {
			this.datasetType = "Unknown";
		}
		sampleReadingsPath = "data" + File.separator + "PA4-" + letter + "-" + this.datasetType + "-SampleReadingsTest.txt";

		this.bodyA = parser.parseBodyData(bodyAPath);
		this.bodyB = parser.parseBodyData(bodyBPath);
		this.sampleReadings = parser.parseSampleReadingsData(sampleReadingsPath, bodyAPath, bodyBPath);
		MeshData meshA = parser.parseMeshData(meshpath);
		this.mesh = new Mesh(meshA);
		this.Freg = new Frame(Matrix.identity(DIM, DIM), new ColumnVector(DIM));
		this.initialize();
		this.iterate();
	}
	private void initialize() {
		this.calcDkVectors();
		this.calcSkVectors();
		this.calcCkVectors();
		this.calcErrors();
	}
	private void iterate() {
		List<Double> avgerrors = new ArrayList<>();
		double threshold = THRESHOLD_STRENGTH*Math3D.average(this.error);
		System.out.println("Threshold:");
		System.out.println(threshold);
		avgerrors.add(Math3D.average(this.error));
		int targetratiocount = 0;
		System.out.printf("Iterating Dataset %c:\n", this.letter);
		for (int i = 0; i < MAX_ITER; i++) {
			this.setAB(threshold);
			Matrix AM = ColumnVector.ColumnVectorListToMatrix(this.A);
			Matrix BM = ColumnVector.ColumnVectorListToMatrix(this.B);
			HornRegistration hr = new HornRegistration(AM, BM, true);
			this.Freg = new Frame(hr.rotationMatrix(), hr.transVector());
			this.calcSkVectors();
			this.calcCkVectors();
			this.calcErrors();
			avgerrors.add(Math3D.average(this.error));
			double curravgerror = avgerrors.get(avgerrors.size() - 1);
			double prevavgerror = avgerrors.get(avgerrors.size() - 2);
			double errorratio = curravgerror/prevavgerror;
			System.out.println("error:");
			System.out.println(curravgerror);
			System.out.println("ratio:");
			System.out.println(errorratio);
			if (errorratio > 0.99 && errorratio <= 1.0 && curravgerror < Math.max(threshold, MIN_THRESHOLD)) {
				targetratiocount++;
				if (targetratiocount == 5) {
					break;
				}
			} else {
				targetratiocount = 0;
			}
		}
	}
	private void setAB(double threshold) {
		double selectionthreshold = 2*threshold/THRESHOLD_STRENGTH;
		A = new ArrayList<>();
		B = new ArrayList<>();
		for (int i = 0; i < this.DkVectors.size(); i++) {
			if (this.error.get(i) < selectionthreshold) {
				this.A.add(this.DkVectors.get(i));
				this.B.add(this.CkVectors.get(i));
			}
		}
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
	private void calcErrors() {
		this.error = new ArrayList<>();
		this.errorvecs = new ArrayList<>();
		for (int i = 0; i < this.SkVectors.size(); i++) {
			ColumnVector errorvector = this.SkVectors.get(i).minus(this.CkVectors.get(i));
			this.errorvecs.add(errorvector);
			this.error.add(errorvector.magnitude());
		}		
	}
	public List<ColumnVector> getErrorVectors() {
		return this.errorvecs;
	}
	public List<Double> getErrors() {
		return this.error;
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
	
	/**
	 * Compares to the expected output for the debug datasets to test whether our output is as expected 
	 */
	public void compareToOutput(PrintWriter writer) {
		DataFileParser parser = new DataFileParser();
		String path = "data" + File.separator + "PA4-" + this.letter + "-Debug-Output.txt";
		Output3Data outputData = parser.parseOutput3Data(path);
		
		System.out.println("Differences in output for dataset " + letter);

		Matrix dkDifference = compareSkToOutput(outputData).transpose();		
		Matrix ckDifference = compareCkToOutput(outputData).transpose();
		Matrix errorDifference = compareErrorsToOutput(outputData).transpose();
		Matrix outputMatrix = MatrixHelper.stackHorizontally(dkDifference, ckDifference, errorDifference);
		outputMatrix.print(writer, 8, 3);
		
		double dkNorm = dkDifference.normF() / dkDifference.getRowDimension();
		double ckNorm = ckDifference.normF() / ckDifference.getRowDimension();
		double errorNorm = errorDifference.normF() / errorDifference.getRowDimension();
		writer.printf("Average norm of error per vector for sk, ck, and differences: %.3f\t%.3f\t%.3f\n", dkNorm, ckNorm, errorNorm);
	}
	
	private Matrix compareSkToOutput(Output3Data outputData) {
		PointCloud expectedDkVectors = new PointCloud(outputData.getdVectors());
		PointCloud actualSkVectors = new PointCloud(this.SkVectors);
		return expectedDkVectors.asMatrix().minus(actualSkVectors.asMatrix());
	}
	
	private Matrix compareCkToOutput(Output3Data outputData) {
		PointCloud expectedCkVectors = new PointCloud(outputData.getcVectors());
		PointCloud actualCkVectors = new PointCloud(this.CkVectors);
		return expectedCkVectors.asMatrix().minus(actualCkVectors.asMatrix());
	}
	
	private Matrix compareErrorsToOutput(Output3Data outputData) {
		Matrix actualErrors = MatrixHelper.convertListToMatrix(this.getErrors());
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
	        		"output" + File.separator + "PA4-" + this.letter + "-" + this.datasetType + "-Output.txt");
	        Matrix skMatrix = new PointCloud(this.SkVectors).asMatrix().transpose();
			Matrix ckMatrix = new PointCloud(this.CkVectors).asMatrix().transpose();
			Matrix errorMatrix = MatrixHelper.convertListToMatrix(this.getErrors()).transpose();
			Matrix outputMatrix = MatrixHelper.stackHorizontally(skMatrix, ckMatrix, errorMatrix);
			outputMatrix.print(writer, 8, 3);
			writer.close();
        } catch (FileNotFoundException e) {
	        e.printStackTrace();
        }		
	}
	
	public static void main(String[] args) throws FileNotFoundException {
		ICPDriver driver;
		for (char letter = 'A'; letter <= 'F'; letter++) {
			driver = new ICPDriver(letter, true);						
			driver.createOutput();
			PrintWriter writer = new PrintWriter("debug" + File.separator + "PA4-" + letter + "-Output-comparison.txt");
			driver.compareToOutput(writer);
			writer.close();
		}
		for (char letter = 'G'; letter <= 'K'; letter++) {
			if (letter == 'I') {
				continue;
			}
			driver = new ICPDriver(letter, false);
			driver.createOutput();
			
		}
	}
}
