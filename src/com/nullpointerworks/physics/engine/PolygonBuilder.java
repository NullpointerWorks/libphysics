package com.nullpointerworks.physics.engine;

import static com.nullpointerworks.physics.engine.VectorMath.sub;
import static com.nullpointerworks.physics.engine.VectorMath.dot;
import static com.nullpointerworks.physics.engine.VectorMath.cross;
import static com.nullpointerworks.physics.engine.VectorMath.normal;
import static com.nullpointerworks.physics.engine.VectorMath.normalize;

import com.nullpointerworks.physics.engine.shape.Polygon;

public class PolygonBuilder 
{
	/**
	 * create a box with the given width and height.<br>
	 */
	public static Shape Box(float width, float height)
	{
		float hw = width*0.5f;
		float hh = height*0.5f;
		float[][] v = { {-hw,-hh},
						{ hw,-hh},
						{ hw, hh},
						{-hw, hh}};
		return Polygon(v);
	}
	
	/**
	 * 
	 */
	public static Shape Polygon(float[][] verts)
	{
		int vertices = verts.length;
		float[][] n = new float[vertices][];
		
		// calculate normals, and convert to unit vectors
		for (int i=0; i<vertices; i++)
		{
			float[] p0 = verts[i];
			float[] p1 = verts[(i+1)%vertices];
			
			float[] norm = sub(p0, p1);
			norm = normal(norm);
			n[i] = normalize(norm);
		}
		
		return new Polygon(verts,n);
	}
	
	/**
	 * make a composition with the shape of the hull around the given set of points.<br>
	 * this shape will always be a convex polygon.
	 */
	public static Composite Hull(float[][] verts)
	{
		Composite c = new Composite();
		int vertices;
		
		// find the most right vertex
		int length = verts.length;
		int most_right = 0;
		float highest_x = verts[0][0];
		for (int i=1; i<length; i++)
		{
			float[] v0 = verts[i];
			
			// get next x
			float x = v0[0]; 
			if (x > highest_x)
			{
				highest_x = x;
				most_right = i;
			}
			else
			if (x == highest_x) // if equal on x
			{
				// get most upper y
				if (v0[1] > verts[most_right][1])
				{
					most_right = i;
				}
			}
		}
		
		// construct hull
		int[] hull = new int[length];
		int v_count = 0;
		int hull_index = most_right;
		
		for (;;)
		{
			hull[v_count] = hull_index;
			
			int next_hull_index = 0;
			for (int i=1; i<length; i++)
			{
				// skip if we pass the same vertex
				if (next_hull_index == hull_index)
				{
					next_hull_index = i;
					continue;
				}

				float[] v_index = verts[i];
				float[] v_next  = verts[next_hull_index];
				float[] v_hull  = verts[hull[v_count]];

				float[] e1 = sub(v_next, v_hull);
				float[] e2 = sub(v_index, v_hull);
				float z = cross(e1, e2);
				
				if (z < 0.0f)
				{
					next_hull_index = i;
				}
				
				// get farthest along the line, of cross is zero
				if (z == 0.0f)
				{
					float m1 = dot(e1, e1);
					float m2 = dot(e2, e2);
					if (m2 > m1)
					{
						next_hull_index = i;
					}
				}
			}
			
			v_count++;
			hull_index = next_hull_index;
			
			// finish algorithm
			if (next_hull_index == most_right)
			{
				vertices = v_count;
				break;
			}
		}
		
		// compile all vertices that comprise the hull
		float[][] v = new float[vertices][];
		float[][] n = new float[vertices][];
		
		// copy vertices
		for (int i=0; i<vertices; i++)
		{
			v[i] = verts[ hull[i] ];
		}
		
		// calculate normals, and convert to unit vectors
		for (int i=0; i<vertices; i++)
		{
			float[] p0 = v[i];
			float[] p1 = v[(i+1)%vertices];
			
			float[] norm = sub(p0, p1);
			norm = normal(norm);
			n[i] = normalize(norm);
		}
		
		c.setShape( new Polygon(v,n) );
		return c;
	}
}
