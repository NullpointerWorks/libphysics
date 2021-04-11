package com.nullpointerworks.physics.engine.collision;

import libMath.vector.Vector2;

import com.nullpointerworks.physics.engine.Composite;
import com.nullpointerworks.physics.engine.Manifold;
import com.nullpointerworks.physics.engine.shape.Circle;
import com.nullpointerworks.physics.engine.shape.Polygon;

import libMath.matrix.Matrix2;

public class CirclePolygon implements ICollisionSolver
{
	public static final CirclePolygon instance = new CirclePolygon();
	
	@Override
	public void solve(Manifold m, Composite A, Composite B) 
	{
		Circle shapeA 	= (Circle)A.shape;
		Polygon shapeB 	= (Polygon)B.shape;
		
		float[][] rotation 	= B.rotation;
		float[][] transpose = Matrix2.transpose(rotation);
		
		float[][] verticesB = shapeB.vertices;
		float[][] normalsB 	= shapeB.normals;
		
		float[] norm,vc;
		float radiusA = shapeA.radius, dist;
		int v_count = verticesB.length;
		
		m.contact_count = 0;
		
		// move circle center vector to the polygon's model space
		float[] center = Matrix2.transform(transpose, Vector2.sub(A.position, B.position) );
		
		// find the face with the penetration penetration
		// normals must be normalized
		int face_normal = 0;
		float separation = 0f;
		for (int i=0; i<v_count; i++)
		{
			float s = Vector2.dot(normalsB[i], Vector2.sub(center, verticesB[i]) );
			if (s > separation)
			{
				separation = s;
				face_normal = i;
			}
		}
		
		// if the biggest positive separation is beyond the radius, return. no collision
		if (separation > radiusA) return;
		
		// check to see if the center of the circle in inside the polygon
		if (separation < ImpulseMath.EPSILON)
		{
			norm = normalsB[face_normal];
			norm = Matrix2.transform(rotation, norm);
			m.normal = Vector2.neg(norm);
			m.contact_count = 1;
			m.penetration = radiusA;
			m.contacts[0] = Vector2.projection(A.position, m.normal, radiusA);
			return;
		}
		
		// get face vertices
		float[] v1 = verticesB[face_normal];
		float[] v2 = verticesB[(face_normal+1)%v_count];
		
		// find the region(voronoi) of the face on the polygon
		
		float dot1 = Vector2.dot( Vector2.sub(center,v1) , Vector2.sub(v2,v1) );
		float dot2 = Vector2.dot( Vector2.sub(center,v2) , Vector2.sub(v1,v2) );
		m.penetration = radiusA - separation;
		m.contact_count = 1;
		
		// dot product vertex projection
		// t = q*a / a*a without the divide. if less than 0, closest on one side
		
		// closest to vertex 1, corner voronoi region
		if (dot1 < 0.0f)
		{
			m.penetration *= 0.1f;
			
			vc = Vector2.sub(v1, center); // get tangent
			if (Vector2.dot(vc, vc) > radiusA*radiusA) return;
			
			// rotate vector to polygon world space
			vc = Matrix2.transform(rotation, vc);
			m.normal = Vector2.normalize(vc);
			
			// rotate vertex to polygon world space
			float[] v = Matrix2.transform(rotation, v1);
			m.contacts[0] = Vector2.add(B.position, v);
			return;
		}
		
		// closest to vertex 2, corner voronoi region
		if (dot2 < 0.0f)
		{
			m.penetration *= 0.1f;
			
			vc = Vector2.sub(v2, center); // get tangent
			if (Vector2.dot(vc, vc) > radiusA*radiusA) return;
			
			// rotate vector to polygon world space
			vc = Matrix2.transform(rotation, vc);
			m.normal = Vector2.normalize(vc);
			
			// rotate vertex to polygon world space
			float[] v = Matrix2.transform(rotation, v2);
			m.contacts[0] = Vector2.add(B.position, v);
			return;
		}
		
		// is inside the voronoi region
		float[] n = normalsB[face_normal];
		
		norm = Vector2.sub(center,v1);
		dist = Vector2.dot(norm , n);
		if (dist > radiusA) return;
		
		norm = Matrix2.transform(rotation, n );
		norm = Vector2.neg(norm);
		m.normal = Vector2.normalize(norm);
		m.contacts[0] = Vector2.projection(A.position, m.normal, radiusA);
	}
}
