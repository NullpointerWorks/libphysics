package com.nullpointerworks.physics.engine.math;

public class MatrixMath 
{

	/**
	 * get the rotation matrix from the given rotations
	 */
	public static float[][] rotation(float angle)
	{
		float cos = (float)StrictMath.cos(angle);
		float sin = (float)StrictMath.sin(angle);
		return new float[][]{{cos, -sin},
							 {sin, cos}};
	}
}
