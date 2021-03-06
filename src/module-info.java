module libnpw.physics 
{
	requires libnpw.math;
	requires libnpw.game;
	requires libnpw.core;
	requires libnpw.color;
	requires libnpw.graphics;
	
	exports com.nullpointerworks.physics.engine;
	exports com.nullpointerworks.physics.engine.collision;
	exports com.nullpointerworks.physics.engine.manifold;
	exports com.nullpointerworks.physics.engine.material;
	exports com.nullpointerworks.physics.engine.motion;
	exports com.nullpointerworks.physics.engine.shape;
}