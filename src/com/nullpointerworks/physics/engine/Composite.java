package com.nullpointerworks.physics.engine;

import java.util.HashMap;


import static com.nullpointerworks.physics.engine.math.MatrixMath.rotation;

import static com.nullpointerworks.physics.engine.math.VectorMath.create;
import static com.nullpointerworks.physics.engine.math.VectorMath.copy;
import static com.nullpointerworks.physics.engine.math.VectorMath.add;
import static com.nullpointerworks.physics.engine.math.VectorMath.cross;
import static com.nullpointerworks.physics.engine.math.VectorMath.project;

public class Composite 
{
	/*
	 * location
	 */
	public float[] position;
	public float[] velocity;
	public float[] force;
	
	/*
	 * angle
	 */
	public float orientation;
	public float angularVelocity;
	public float torque;
	
	/*
	 * mass
	 */
	public float mass;
	public float inertia;
	public float inv_mass;
	public float inv_inertia;
	
	/*
	 * composition
	 */
	private Material material;
	private Shape shape;
	
	/*
	 * other data
	 */
	public float[][] rotation;
	public boolean immovable;
	public HashMap<Composite,Integer> ignore;
	
	public Composite()
	{
		position 	= create(0f, 0f);
		velocity 	= create(0f, 0f);
		force 		= create(0f, 0f); // gets reset per update
		
		rotation 		= rotation(0f);
		orientation 	= 0f;
		angularVelocity = 0f;
		torque 			= 0f; // gets reset per update
		
		immovable 	= false;
		material 	= Material.Medium();
		shape 		= null;
		ignore 		= new HashMap<Composite,Integer>();
	}
	
	/**
	 * 
	 * @return
	 */
	public Shape getShape() 
	{
		return shape;
	}
	
	/**
	 * 
	 * @return
	 */
	public Material getMaterial() 
	{
		return material;
	}
	
	/**
	 * 
	 * @return a copy of this composite
	 */
	public Composite getCopy()
	{
		Composite c = new Composite();
		
		c.position = copy(position);
		c.velocity = copy(velocity);
		c.force = copy(force);
		
		c.rotation = rotation(orientation);
		c.orientation = orientation;
		c.angularVelocity = angularVelocity;
		c.torque = torque;
		
		c.immovable = immovable;
		c.material = material;
		c.shape = shape;
		c.ignore = ignore;
		
		return c;
	}
	
	/**
	 * set a material for this composite
	 */
	public Composite setMaterial(Material mat)
	{
		if (immovable) return this;
		material = mat;
		compute();
		return this;
	}
	
	/**
	 * provide a shape for the composite
	 */
	public Composite setShape(Shape sh)
	{
		shape = sh;
		compute();
		return this;
	}
	
	/**
	 * set this object as immovable by the engine
	 */
	public Composite setImmovable(boolean immovable) 
	{
		this.material = Material.Static();
		this.immovable = immovable;
		compute();
		return this;
	}
	
	/**
	 * 
	 */
	public Composite setBlacklist(Composite c)
	{
		if (ignore.containsKey(c)) return this;
		ignore.put(c, ignore.size());
		return this;
	}

	/**
	 * set the location for this body
	 */
	public Composite setPosition(float x, float y) 
	{
		position = create(x, y);
		return this;
	}
	
	/**
	 * set the composite to a specific angle
	 */
	public Composite setOrientation(float angle)
	{
		rotation = rotation(angle);
		orientation = angle;
		return this;
	}
	
	// =================================================
	
	/**
	 * check to see if the Composite has been blacklisted for collision checks
	 */
	public boolean contains(Composite c)
	{
		return ignore.containsKey(c);
	}
	
	/**
	 * add the given force vector to the body
	 */
	public void applyForce(float[] f)
	{
		force = add(force, f);
	}
	
	/**
	 * velocity = impulse / mass<br>
	 * aVelocity = (contact x impulse) / inertia
	 */
	public void applyImpulse(float[] impulse, float[] contact)
	{
		velocity 		= project(velocity, impulse, inv_mass);
		angularVelocity = angularVelocity + ( inv_inertia * cross(contact, impulse) );
	}
	
	/**
	 * clear forces from this object
	 */
	public void clear() 
	{
		force 	= create(0f, 0f);
		torque 	= 0f;
	}
	
	// ====================================================
	
	/*
	 * compute mass and moment of inertia
	 */
	private void compute() 
	{
		if (shape != null)
		{
			float density = material.getDensity();
			setMass(shape.getMass(density) );
			setInertia(shape.getInertia(density) );
		}
	}
	
	/**
	 * calculate the mass<br>
	 * d = density<br>
	 * V = volume<br>
	 * mass = density * volume;
	 */
	private void setMass(float m)
	{
		mass = m;
		inv_mass = (m==0.0f)? 0.0f: 1f/m;
	}
	
	/**
	 * set the moment of inertia. This value comes from the Shape class<br>
	 * Ip = SUM(m*r^2)<br>
	 */
	private void setInertia(float Ip)
	{
		inertia = Ip;
		inv_inertia = (Ip==0.0f)? 0.0f: 1f/Ip;
	}
}
