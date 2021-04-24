package com.nullpointerworks.physics.engine;

import com.nullpointerworks.physics.engine.collision.*;

public class Collision 
{
	/*
	 * list of collision cases
	 */
	private static CollisionSolver[][] solvers =
	{
		{new CircleCircle(), new CirclePolygon()},
		{new PolygonCircle(), new PolygonPolygon()}
	};
	
	/*
	 * 
	 */
	public static void solve(Manifold m, Composite A, Composite B)
	{
		int a = A.getShape().getType().ordinal();
		int b = B.getShape().getType().ordinal();
		solvers[a][b].solve(m, A, B);
	}
}
