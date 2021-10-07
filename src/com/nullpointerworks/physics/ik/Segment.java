package com.nullpointerworks.physics.ik;

public interface Segment 
{
	void setParent(Segment p);
	
	float[] getBase();
	float[] getDest();
	float getAngle();
	Segment getParent();
	
	void follow(float[] target);
	void follow(Segment target);
	void position(float[] delta);
}
