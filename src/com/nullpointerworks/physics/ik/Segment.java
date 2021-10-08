package com.nullpointerworks.physics.ik;

public interface Segment 
{
	void setBase(float[] b);
	void setMagnitude(float l);
	void setAngle(float a);
	void setParent(Segment p);
	
	float[] getBase();
	float[] getDest();
	float getMagnitude();
	float getAngle();
	Segment getParent();
	
	void follow(float[] target);
	void follow(Segment target);
	void position(float[] delta);
}
