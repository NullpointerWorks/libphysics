package com.nullpointerworks.physics.engine;

import com.nullpointerworks.physics.engine.collision.ICollisionSolver;

public class Collision 
{
	/*
	 * list of collision cases
	 */
	private static ICollisionSolver[][] solvers =
	{
		{CircleCircle.instance, CirclePolygon.instance},
		{PolygonCircle.instance, PolygonPolygon.instance}
	};
	
	/*
	 * 
	 */
	public static void Test(Manifold m, Composite A, Composite B)
	{
		int a = A.shape.type().ordinal();
		int b = B.shape.type().ordinal();
		solvers[a][b].solve(m, A, B);
	}
}
