package com.nullpointerworks.physics.engine.collision;

import static com.nullpointerworks.physics.engine.MatrixMath.rotation;
import static com.nullpointerworks.physics.engine.MatrixMath.transform;
import static com.nullpointerworks.physics.engine.MatrixMath.transpose;

import static com.nullpointerworks.physics.engine.VectorMath.add;
import static com.nullpointerworks.physics.engine.VectorMath.sub;
import static com.nullpointerworks.physics.engine.VectorMath.dot;
import static com.nullpointerworks.physics.engine.VectorMath.neg;
import static com.nullpointerworks.physics.engine.VectorMath.project;
import static com.nullpointerworks.physics.engine.VectorMath.magnitude;
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
		float[] positionA 		= A.getLinearMotion().getPosition();
		float radiusA 			= shapeA.getRadius();

		Polygon shapeB 			= (Polygon)B.getShape();
		float[] positionB 		= B.getLinearMotion().getPosition();
		float orientationB 		= B.getAngularMotion().getOrientation();
		float[][] verticesB 	= shapeB.getVertices();
		float[][] normalsB 		= shapeB.getNormals();
		
		float[][] rotationB 	= rotation(orientationB);
		float[][] transpose 	= transpose(rotationB);
		
		// move circle's position relative to polygon's center to the polygon's model space
		float[] relativeCirclePos = sub(positionA, positionB);
		float[] center = transform(transpose, relativeCirclePos);
		
		// find the face with the closest penetration
		// normals must be normalized
		int face_normal = 0;
		float separation = 0f;
		int v_count = verticesB.length;
		for (int i=0; i<v_count; i++)
		{
			float[] vert = verticesB[i]; // vertex in the polygon
			float[] norm = normalsB[i]; // normal pointing to the next vertex
			
			float s = dot( norm, sub(center, vert) );
			
			if (s > separation)
			{
				separation = s;
				face_normal = i;
			}
		}
		
		m.contact_count = 0;
		
		// if the biggest positive separation is beyond the radius
		// no collision
		if (separation > radiusA) 
		{
			return;
		}
		
		// check to see if the center of the circle is inside the polygon
		// extreme collision case
		if (separation < 0f)
		{
			float[] norm = normalsB[face_normal];
			norm = transform(rotationB, norm);
			m.normal = neg(norm);
			m.contact_count = 1;
			m.penetration = radiusA;
			m.contacts[0] = project(positionA, m.normal, radiusA);
			return;
		}
		
		// collision
		if (separation < radiusA) 
		{
			// get face vertices
			float[] v1 = verticesB[face_normal];
			float[] v2 = verticesB[(face_normal+1)%v_count];
			
			m.contact_count = 1;
			
			// dot product vertex projection
			// t = q*a / a*a without the divide. if less than 0, closest on one side
			
			// first check to see if the circle collides on a corner of the polygon
			
			// closest to vertex 1, corner voronoi region
			float dot1 = dot( sub(center,v1) , sub(v2,v1) );
			if (dot1 < 0.0f)
			{
				float[] vc = sub(v1, center); // get reverse
				if (dot(vc, vc) > radiusA*radiusA) return;
				
				// rotate vector to polygon world space
				vc = transform(rotationB, vc);
				
				// rotate vertex to polygon world space
				float[] v = transform(rotationB, v1);
				
				m.normal = normalize(vc);
				m.contacts[0] = add(positionB, v);
				m.penetration = radiusA - separation; //magnitude( sub(center,v1) );
				return;
			}
			
			// closest to vertex 2, corner voronoi region
			float dot2 = dot( sub(center,v2) , sub(v1,v2) );
			if (dot2 < 0.0f)
			{
				float[] vc = sub(v2, center); // get reverse
				if (dot(vc, vc) > radiusA*radiusA) return;
				
				// rotate vector to polygon world space
				vc = transform(rotationB, vc);
				
				// rotate vertex to polygon world space
				float[] v = transform(rotationB, v2);
				
				m.normal = normalize(vc);
				m.contacts[0] = add(positionB, v);
				m.penetration = radiusA - separation; //magnitude( sub(center,v2) );
				return;
			}
			
			// is inside the voronoi region
			// in other words, the circle is hitting a face of the polygon
			float[] n 		= normalsB[face_normal];
			float[] norm 	= sub(center,v1);
			float dist 		= dot(norm, n);
			if (dist > radiusA) return;
			
			norm = transform(rotationB, n);
			norm = neg(norm);
			m.normal = normalize(norm);
			m.contacts[0] = project(positionA, m.normal, radiusA);
			m.penetration = radiusA - separation;
		}
	}
}
