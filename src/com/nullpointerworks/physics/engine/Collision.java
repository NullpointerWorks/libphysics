package com.nullpointerworks.physics.engine;

import com.nullpointerworks.physics.engine.collision.*;
import com.nullpointerworks.physics.engine.collision.ICollisionSolver;

public class Collision 
{
	/*
	 * list of collision cases
	 */
	private static ICollisionSolver[][] solvers =
	{
		{new CircleCircle(), new CirclePolygon()},
		{new PolygonCircle(), new PolygonPolygon()}
	};
	
	/*
	 * 
	 */
	public static void solve(Manifold m, Composite A, Composite B)
	{
		int a = A.shape.type().ordinal();
		int b = B.shape.type().ordinal();
		solvers[a][b].solve(m, A, B);
	}
}
