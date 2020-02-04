package cis.pa3.tools;

import java.util.List;

import Jama.Matrix;

public class MatrixHelper {
	
	/**
	 * Stacks matrices vertically. Fills in with zeros if rows are not all the same
	 * @param matrices All matrices to stack
	 * @return The stacked matrix
	 */
	public static Matrix stackVertically(Matrix... matrices) {
		int totalRows = 0, totalCols = 0;
		for (Matrix matrix : matrices) {
			totalRows += matrix.getRowDimension();
			totalCols  = Math.max(totalCols, matrix.getColumnDimension());
		}
		Matrix finalMatrix = new Matrix(totalRows, totalCols);
		int rowOffset = 0;
		for (Matrix matrix : matrices) {
			int currCol = matrix.getColumnDimension();
			int currRow = matrix.getRowDimension();
			finalMatrix.setMatrix(rowOffset, rowOffset + currRow - 1, 0, currCol - 1, matrix);
			rowOffset += matrix.getRowDimension();
		}
		return finalMatrix;
		
	}
	
	/**
	 * Stacks matrices horizontally. Fills in with zeros if rows are not all the same
	 * @param matrices All matrices to stack
	 * @return The stack matrix
	 */
	public static Matrix stackHorizontally(Matrix... matrices) {
		int totalRows = 0, totalCols = 0;
		for (Matrix matrix : matrices) {
			totalRows  = Math.max(totalRows, matrix.getRowDimension());
			totalCols += matrix.getColumnDimension();

		}
		Matrix finalMatrix = new Matrix(totalRows, totalCols);
		int colOffset = 0;
		for (Matrix matrix : matrices) {
			int currCol = matrix.getColumnDimension();
			int currRow = matrix.getRowDimension();
			finalMatrix.setMatrix(0, currRow - 1, colOffset, colOffset + currCol - 1, matrix);
			colOffset += matrix.getColumnDimension();
		}
		return finalMatrix;
		
	}
	
	public static Matrix convertListToMatrix(List<Double> list) {
		double[] array = new double[list.size()];
		for (int i = 0; i < list.size(); i++) {
			array[i] = list.get(i);
		}
		return new Matrix(array, 1);
	}
}
