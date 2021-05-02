package com.nullpointerworks.physics.engine.collision;

import static com.nullpointerworks.physics.engine.MatrixMath.rotation;
import static com.nullpointerworks.physics.engine.MatrixMath.transform;
import static com.nullpointerworks.physics.engine.MatrixMath.transpose;

import static com.nullpointerworks.physics.engine.VectorMath.add;
import static com.nullpointerworks.physics.engine.VectorMath.sub;
import static com.nullpointerworks.physics.engine.VectorMath.dot;
import static com.nullpointerworks.physics.engine.VectorMath.neg;
import static com.nullpointerworks.physics.engine.VectorMath.project;
import static com.nullpointerworks.physics.engine.VectorMath.normalize;
import static com.nullpointerworks.physics.engine.VectorMath.normal;

import com.nullpointerworks.physics.engine.CollisionSolver;
import com.nullpointerworks.physics.engine.Composite;
import com.nullpointerworks.physics.engine.manifold.Manifold;
import com.nullpointerworks.physics.engine.shape.Circle;
import com.nullpointerworks.physics.engine.shape.Polygon;

public class CirclePolygon implements CollisionSolver
{
	@Override
	public void solve(Manifold m, Composite A, Composite B) 
	{
		Circle shapeA 			= (Circle)A.getShape();
		Polygon shapeB 			= (Polygon)B.getShape();
		
		float[] Aposition 		= A.getLinearMotion().getPosition();
		float[] Bposition 		= B.getLinearMotion().getPosition();
		float Borientation 		= B.getAngularMotion().getOrientation();
		
		float Aradius 			= shapeA.getRadius();
		float[][] Bvertices 	= shapeB.getVertices();
		float[][] Bnormals 		= shapeB.getNormals();
		
		float[][] rotation 		= rotation(Borientation);
		float[][] transpose 	= transpose(rotation);
		
		float dist;
		int v_count = Bvertices.length;
		
		m.contact_count = 0;
		
		// move circle center vector to the polygon's model space
		float[] center = transform(transpose, sub(Aposition, Bposition) );
		
		// find the face with the closest penetration
		// normals must be normalized
		int face_normal = 0;
		float separation = 0f;
		for (int i=0; i<v_count; i++)
		{
			float[] vert = Bvertices[i];
			float[] norm = Bnormals[i];
			
			float s = dot(norm, sub(center, vert) );
			if (s > separation)
			{
				separation = s;
				face_normal = i;
			}
		}
		
		// if the biggest positive separation is beyond the radius, return. no collision
		if (separation > Aradius) 
		{
			return;
		}

		float[] norm,vc;
		
		// check to see if the center of the circle is inside the polygon
		if (separation < 0f)
		{
			norm = Bnormals[face_normal];
			norm = transform(rotation, norm);
			m.normal = neg(norm);
			m.contact_count = 1;
			m.penetration = Aradius;
			m.contacts[0] = project(Aposition, m.normal, Aradius);
			return;
		}
		
		// get face vertices
		float[] v1 = Bvertices[face_normal];
		float[] v2 = Bvertices[(face_normal+1)%v_count];
		
		// find the region(voronoi) of the face on the polygon
		
		float dot1 = dot( sub(center,v1) , sub(v2,v1) );
		float dot2 = dot( sub(center,v2) , sub(v1,v2) );
		m.penetration = Aradius - separation;
		m.contact_count = 1;
		
		// dot product vertex projection
		// t = q*a / a*a without the divide. if less than 0, closest on one side
		
		// closest to vertex 1, corner voronoi region
		if (dot1 < 0.0f)
		{
			m.penetration *= 0.1f;
			
			vc = sub(v1, center); // get tangent
			if (dot(vc, vc) > Aradius*Aradius) return;
			
			// rotate vector to polygon world space
			vc = transform(rotation, vc);
			m.normal = normalize(vc);
			
			// rotate vertex to polygon world space
			float[] v = transform(rotation, v1);
			m.contacts[0] = add(Bposition, v);
			return;
		}
		
		// closest to vertex 2, corner voronoi region
		if (dot2 < 0.0f)
		{
			m.penetration *= 0.1f;
			
			vc = sub(v2, center); // get tangent
			if (dot(vc, vc) > Aradius*Aradius) return;
			
			// rotate vector to polygon world space
			vc = transform(rotation, vc);
			m.normal = normalize(vc);
			
			// rotate vertex to polygon world space
			float[] v = transform(rotation, v2);
			m.contacts[0] = add(Bposition, v);
			return;
		}
		
		// is inside the voronoi region
		float[] n = Bnormals[face_normal];
		
		norm = sub(center,v1);
		dist = dot(norm , n);
		if (dist > Aradius) return;
		
		norm = transform(rotation, n );
		norm = neg(norm);
		m.normal = normalize(norm);
		m.contacts[0] = project(Aposition, m.normal, Aradius);
	}
}
