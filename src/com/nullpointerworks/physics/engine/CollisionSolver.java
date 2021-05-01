package com.nullpointerworks.physics.engine;

import com.nullpointerworks.physics.engine.manifold.Manifold;

public interface CollisionSolver 
{
	public void solve( Manifold m, Composite A, Composite B );
}
