package com.nullpointerworks.physics.engine;

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
	
	public static float[][] transpose(float[][] m)
	{
		float[] r0 = {m[0][0],m[1][0]};
		float[] r1 = {m[0][1],m[1][1]};
		return new float[][] {r0,r1};
	}

	public static float[] transform(float[][] m, float[] v)
	{
		float[] m0 = m[0];
		float[] m1 = m[1];
		float vx = m0[0]*v[0] + m0[1]*v[1];
		float vy = m1[0]*v[0] + m1[1]*v[1];
		return new float[] {vx,vy};
	}

	public static float[][] transform(float[][] matrix, float[][] vdata)
	{
		float[][] verts = new float[vdata.length][2];
		for (int i=0,l=vdata.length; i<l; i++)
		{
			float[] v = vdata[i];
			verts[i] = transform(matrix,v);
		}
		return verts;
	}
	
	
}
