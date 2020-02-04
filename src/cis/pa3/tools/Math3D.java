package cis.pa3.tools;

import java.util.List;


public class Math3D {
	private static final int DIM = 3;
	public static double dotProduct(ColumnVector c1, ColumnVector c2) {
		double prod = 0;
		for (int i = 0; i < DIM; i++) {
			prod += c1.get(i) * c2.get(i);
		}
		return prod;
	}
	public static double max(List<Double> ld) {
		if (ld.size() < 1) {
			System.err.println("List is empty.");
			System.exit(1);
		}
		double max = ld.get(0);
		for (int i = 1; i < ld.size(); i++) {
			if (ld.get(i) > max) {
				max = ld.get(i);
			}
		}
		return max;
	}
	public static double average(List<Double> ld) {
		if (ld.size() < 1) {
			System.err.println("List is empty.");
			System.exit(1);
		}
		double avg = 0;
		for (int i = 0; i < ld.size(); i++) {
			avg += ld.get(i);
		}
		avg = avg/ld.size();
		return avg;
	}
}