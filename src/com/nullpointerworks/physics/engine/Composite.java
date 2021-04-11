package com.nullpointerworks.physics.engine;

import java.util.HashMap;

import com.nullpointerworks.math.vector.Vector2;
import com.nullpointerworks.physics.engine.shape.Shape;

public class Composite 
{
	private final Vector2 vec2 = Vector2.New();
	
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
	public Material material;
	public Shape shape;
	
	/*
	 * other data
	 */
	public float[][] rotation;
	public boolean immovable;
	public HashMap<Composite,Integer> ignore;
	
	public Composite()
	{
		position 	= vec2.New(0f, 0f);
		velocity 	= vec2.New(0f, 0f);
		force 		= vec2.New(0f, 0f); // gets reset per update
		
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
	 * returns a new empty composite
	 */
	public Composite New()
	{
		return new Composite();
	}
	
	/**
	 * returns a copy of this composite
	 */
	public Composite getCopy()
	{
		Composite c = New();

		c.position = vec2.copy(position);
		c.velocity = vec2.copy(velocity);
		c.force = vec2.copy(force);
		
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
		position = vec2.New(x, y);
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
		force = vec2.add(force, f);
	}
	
	/**
	 * velocity = impulse / mass<br>
	 * aVelocity = (contact x impulse) / inertia
	 */
	public void applyImpulse(float[] impulse, float[] contact)
	{
		velocity 		= vec2.project(velocity, impulse, inv_mass);
		angularVelocity = angularVelocity + ( inv_inertia * vec2.cross(contact, impulse)[2] );
	}
	
	/**
	 * clear forces from this object
	 */
	public void clear() 
	{
		force 	= vec2.New(0f, 0f);
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
			float density = material.density;
			setMass(shape.mass(density) );
			setInertia(shape.inertia(density) );
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
	
	/**
	 * get the rotation matrix from the given rotations
	 */
	private float[][] rotation(float angle)
	{
		float cos = (float)StrictMath.cos(angle);
		float sin = (float)StrictMath.sin(angle);
		return new float[][]{{cos, -sin},
							 {sin, cos}};
	}
}
