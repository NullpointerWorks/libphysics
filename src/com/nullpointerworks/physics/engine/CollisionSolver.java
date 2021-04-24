package com.nullpointerworks.physics.engine;

public interface CollisionSolver 
{
	public void solve( Manifold m, Composite A, Composite B );
}
