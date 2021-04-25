package com.nullpointerworks.physics.engine.collision;

import com.nullpointerworks.physics.engine.CollisionSolver;
import com.nullpointerworks.physics.engine.Composite;
import com.nullpointerworks.physics.engine.ImpulseMath;
import com.nullpointerworks.physics.engine.Manifold;
import com.nullpointerworks.physics.engine.MatrixMath;
import com.nullpointerworks.physics.engine.VectorMath;
import com.nullpointerworks.physics.engine.shape.Circle;
import com.nullpointerworks.physics.engine.shape.Polygon;

public class CirclePolygon implements CollisionSolver
{
	@Override
	public void solve(Manifold m, Composite A, Composite B) 
	{
		Circle shapeA 	= (Circle)A.getShape();
		Polygon shapeB 	= (Polygon)B.getShape();
		
		float[][] rotation 	= B.rotation;
		float[][] transpose = MatrixMath.transpose(rotation);
		
		float[][] verticesB = shapeB.vertices;
		float[][] normalsB 	= shapeB.normals;
		
		float[] norm,vc;
		float radiusA = shapeA.getRadius(), dist;
		int v_count = verticesB.length;

		float[] Aposition = A.getLinearMotion().getPosition();
		float[] Bposition = B.getLinearMotion().getPosition();
		
		m.contact_count = 0;
		
		// move circle center vector to the polygon's model space
		float[] center = MatrixMath.transform(transpose, VectorMath.sub(Aposition, Bposition) );
		
		// find the face with the penetration penetration
		// normals must be normalized
		int face_normal = 0;
		float separation = 0f;
		for (int i=0; i<v_count; i++)
		{
			float s = VectorMath.dot(normalsB[i], VectorMath.sub(center, verticesB[i]) );
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
			norm = MatrixMath.transform(rotation, norm);
			m.normal = VectorMath.neg(norm);
			m.contact_count = 1;
			m.penetration = radiusA;
			m.contacts[0] = VectorMath.project(Aposition, m.normal, radiusA);
			return;
		}
		
		// get face vertices
		float[] v1 = verticesB[face_normal];
		float[] v2 = verticesB[(face_normal+1)%v_count];
		
		// find the region(voronoi) of the face on the polygon
		
		float dot1 = VectorMath.dot( VectorMath.sub(center,v1) , VectorMath.sub(v2,v1) );
		float dot2 = VectorMath.dot( VectorMath.sub(center,v2) , VectorMath.sub(v1,v2) );
		m.penetration = radiusA - separation;
		m.contact_count = 1;
		
		// dot product vertex projection
		// t = q*a / a*a without the divide. if less than 0, closest on one side
		
		// closest to vertex 1, corner voronoi region
		if (dot1 < 0.0f)
		{
			m.penetration *= 0.1f;
			
			vc = VectorMath.sub(v1, center); // get tangent
			if (VectorMath.dot(vc, vc) > radiusA*radiusA) return;
			
			// rotate vector to polygon world space
			vc = MatrixMath.transform(rotation, vc);
			m.normal = VectorMath.normalize(vc);
			
			// rotate vertex to polygon world space
			float[] v = MatrixMath.transform(rotation, v1);
			m.contacts[0] = VectorMath.add(Bposition, v);
			return;
		}
		
		// closest to vertex 2, corner voronoi region
		if (dot2 < 0.0f)
		{
			m.penetration *= 0.1f;
			
			vc = VectorMath.sub(v2, center); // get tangent
			if (VectorMath.dot(vc, vc) > radiusA*radiusA) return;
			
			// rotate vector to polygon world space
			vc = MatrixMath.transform(rotation, vc);
			m.normal = VectorMath.normalize(vc);
			
			// rotate vertex to polygon world space
			float[] v = MatrixMath.transform(rotation, v2);
			m.contacts[0] = VectorMath.add(Bposition, v);
			return;
		}
		
		// is inside the voronoi region
		float[] n = normalsB[face_normal];
		
		norm = VectorMath.sub(center,v1);
		dist = VectorMath.dot(norm , n);
		if (dist > radiusA) return;
		
		norm = MatrixMath.transform(rotation, n );
		norm = VectorMath.neg(norm);
		m.normal = VectorMath.normalize(norm);
		m.contacts[0] = VectorMath.project(Aposition, m.normal, radiusA);
	}
}
