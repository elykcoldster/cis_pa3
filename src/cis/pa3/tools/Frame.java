package cis.pa3.tools;
import Jama.Matrix;

/**
 * Defines a frame by its rotation matrix and translation vector
 * 
 * @author John Lee, Kyle Xiong
 *
 */
public class Frame {
	private Matrix rotationMatrix;
	private ColumnVector translationVector;
	
	public Frame(Matrix R, ColumnVector p) {
		this.rotationMatrix = R;
		this.translationVector = p;
	}
	public Matrix getRotationMatrix() {
		return this.rotationMatrix;
	}
	public ColumnVector getTranslationVector() {
		return this.translationVector;
	}
	
	/**
	 * Combines frames in the order: (this)(frames[0])(frames[1])...(frames[n])
	 * and sets it to this frame
	 * 
	 * @param frames
	 *            The frames in order of transformation
	 */
	public Frame combineFrames(Frame... frames) {
		for (Frame frame : frames) {
			Matrix R1 = this.getRotationMatrix();
			Matrix R2 = frame.getRotationMatrix();
			ColumnVector p1 = this.getTranslationVector();
			ColumnVector p2 = frame.getTranslationVector();
			this.translationVector = p1.plus(new ColumnVector(R1.times(p2
			        .getMatrix())));
			this.rotationMatrix = R1.times(R2);
		}
		return this;
	}
	
	/**
	 * Returns an inverse of the current frame
	 * 
	 * @return The inverse frame
	 */
	public Frame inverse() {
		Matrix inverseRotation = this.rotationMatrix.inverse();
		return new Frame(inverseRotation, new ColumnVector(inverseRotation
		                .times(-1).times(this.translationVector.getMatrix())));
	}
	
	/**
	 * Transforms the given vector into this frame
	 * 
	 * @param vector
	 *            The initial vector
	 * @return The vector in the new frame
	 */
	public ColumnVector timesVector(ColumnVector vector) {
		Matrix rotationTimesVector = this.rotationMatrix.times(vector.getMatrix());
		return new ColumnVector(rotationTimesVector.plus(this.translationVector.getMatrix()));
	}

}