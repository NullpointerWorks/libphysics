package com.nullpointerworks.physics.engine.shape;

import com.nullpointerworks.math.vector.Vector2;

public class Polygon extends Shape 
{
	private final Vector2 vec2 = Vector2.New();
	private float area;
	private float I;
	public float[][] vertices;
	public float[][] normals;
	
	/**
	 * convex polygon
	 */
	public Polygon(float[][] vertices, float[][] normals)
	{
		this.vertices = vertices;
		this.normals = normals;
		area = I = 0f;
		calculate();
	}
	
	/**
	 * quick copy/clone constructor
	 */
	private Polygon(float[][] vertices, float[][] normals, float area, float I)
	{
		this.vertices = vertices;
		this.normals = normals;
		this.area = area;
		this.I = I;
	}
	
	/*
	 * calculate area and moment of inertia
	 */
	private void calculate()
	{
		float inv_12 = 1.0f / 12.0f;
		for (int i=0,l=vertices.length; i<l; i++)
		{
			// get triangle area assume third point is V(0,0)
			float[] p0 = vertices[i];
			float[] p1 = vertices[(i+1)%l];
			float z = vec2.cross(p0, p1)[2];
			
			// collect area squares
			float xy2 = vec2.dot(p0,p0) + vec2.dot(p0,p1) + vec2.dot(p1,p1);
			I += inv_12 * z * xy2;
			area += z;
		}
		area *= 0.5f;
	}
	
	@Override
	public float area() 
	{
		return area;
	}
	
	@Override
	public float mass(float d) 
	{
		return area * d;
	}
	
	@Override
	public float inertia(float d) 
	{
		return I * d;
	}
	
	@Override
	public ShapeType type() 
	{
		return ShapeType.Poly;
	}
	
	@Override
	public Shape clone() 
	{
		return new Polygon(vertices,normals,area,I);
	}
}
