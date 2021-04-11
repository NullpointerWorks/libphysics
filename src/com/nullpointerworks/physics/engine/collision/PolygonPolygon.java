package com.nullpointerworks.physics.engine.collision;

import com.nullpointerworks.physics.engine.Composite;
import com.nullpointerworks.physics.engine.ImpulseMath;
import com.nullpointerworks.physics.engine.Manifold;
import com.nullpointerworks.physics.engine.shape.Polygon;

import libMath.matrix.Matrix2;
import libMath.vector.Vector2;

public class PolygonPolygon implements ICollisionSolver 
{
	public static final PolygonPolygon instance = new PolygonPolygon();
	
	/*
	 * Gilbert-Johnson-Keerthi (GJK) algorithm
	 */
	@Override
	public void solve(Manifold m, Composite A, Composite B)
	{
		m.contact_count = 0;
		
		// find face-to-face penetration axes
		int[] face_a = {0};
		float penetrationA = LeastPenetrationAxes(face_a, A,B);
		if (penetrationA >= 0.0f) return;
		
		int[] face_b = {0};
		float penetrationB = LeastPenetrationAxes(face_b, B,A);
		if (penetrationB >= 0.0f) return;
		
		// find the face of incidence
		if (ImpulseMath.gt(penetrationA, penetrationB))
		{
			solve(m,A,B, face_a[0],false);
		}
		else
		{
			solve(m,B,A, face_b[0],true);
		}
	}
	
	/*
	 * solve continuation
	 */
	public void solve(Manifold m, Composite R, Composite I, int reference_index, boolean handedness)
	{
		Polygon shapeR 	= (Polygon)R.shape;
		
		float[][] incFace = new float[2][];
		IncidentFace(incFace, R, I, reference_index);
		
		float[][] refVertices	= shapeR.vertices;
		float[][] refMatrix 	= R.rotation;
		int l = refVertices.length;
		
		// get vertices from the reference polygon
		float[] v1 = refVertices[ reference_index ];
		float[] v2 = refVertices[(reference_index+1)%l ];
		
		// translate to world space
		v1 = Matrix2.transform(refMatrix, v1);
		v2 = Matrix2.transform(refMatrix, v2);
		v1 = Vector2.add(v1, R.position);
		v2 = Vector2.add(v2, R.position);
		
		// get normal and tangent of the world-space face
		float[] plane_normal 	= Vector2.sub(v2, v1);
		plane_normal 			= Vector2.normalize(plane_normal);
		float[] face_normal 	= Vector2.normal(plane_normal, -1f);
		
		// find collision normal
		float refC 		= Vector2.dot(face_normal, v1);
		float neg_side 	= -Vector2.dot(plane_normal, v1);
		float pos_side 	= Vector2.dot(plane_normal, v2);
		
		if ( clip(Vector2.neg(plane_normal), neg_side, incFace ) < 2 ) 
			return;
		
		if ( clip(plane_normal, pos_side, incFace ) < 2 ) 
			return;
		
		// flip collision direction normal
		m.normal = face_normal;
		if (handedness)
		{
			m.normal = Vector2.neg(m.normal);
		}
		
		// find separation
		int cp = 0;
		float separation1 = Vector2.dot(face_normal, incFace[0]) - refC;
		float separation2 = Vector2.dot(face_normal, incFace[1]) - refC;
		
		m.penetration = 0.0f;
		if (separation1 <= 0.0f)
		{
			m.contacts[cp] = Vector2.copy(incFace[0]);
			m.penetration += -separation1;
			cp++;
		}
		
		if (separation2 <= 0.0f)
		{
			m.contacts[cp] = Vector2.copy(incFace[1]);
			m.penetration += -separation2;
			cp++;
			
			m.penetration = m.penetration / (float)cp;
		}

		m.contact_count = cp;
	}
	
	// =========================================================================
	
	/*
	 * 
	 */
	private float LeastPenetrationAxes(int[] faceIndex, Composite A, Composite B)
	{
		Polygon shapeA 	= (Polygon)A.shape;
		Polygon shapeB 	= (Polygon)B.shape;
		
		float best_dist = -Float.MAX_VALUE;
		int best_index 	= 0;

		float[][] verticesA	= shapeA.vertices;
		float[][] verticesB	= shapeB.vertices;
		float[][] normals 	= shapeA.normals;
		
		float[][] rotationA = A.rotation;
		float[][] rotationB = B.rotation;
		float[][] trnsposeB = Matrix2.transpose(rotationB);

		float[] posA = A.position;
		float[] posB = B.position;
		
		for (int i=0,l=verticesA.length; i<l; i++)
		{
			float[] vertex = verticesA[i];
			float[] normal = normals[i];
			
			// move normal to polygon B's model space
			float[] nw = Matrix2.transform(rotationA, normal);
			float[] n = Matrix2.transform(trnsposeB, nw);
			
			// move vertex from model space A to model space B
			float[] v = Matrix2.transform(rotationA, vertex);
			v = Vector2.add(v, posA);
			v = Vector2.sub(v, posB);
			v = Matrix2.transform(trnsposeB, v);
			
			// get support
			float[] s = support(verticesB, Vector2.neg(n) );
			float dist = Vector2.dot(n, Vector2.sub(s, v) );
			if (dist > best_dist)
			{
				best_dist = dist;
				best_index = i;
			}
		}
		
		faceIndex[0] = best_index;
		return best_dist;
	}
	
	/**
	 * support function for Gilbert-Johnson-Keerthi (GJK) algorithm<br>
	 * www.toptal.com/game/video-game-physics-part-ii-collision-detection-for-solid-objects
	 */
	private float[] support(float[][] vertices, float[] dir) 
	{
		float bestProj = -Float.MAX_VALUE;
		float[] best_vertex = null;
		
		for (int i=0, l=vertices.length; i<l; i++)
		{
			float[] v = vertices[i];
			float proj = Vector2.dot(v, dir);
			
			if (proj > bestProj)
			{
				best_vertex = v;
				bestProj = proj;
			}
		}
		return best_vertex;
	}
	
	/*
	 * 
	 */
	private void IncidentFace(float[][] v, Composite R, Composite I, int index)
	{
		Polygon refPoly = (Polygon)R.shape;
		Polygon incPoly = (Polygon)I.shape;
		
		float[][] refMatrix 	= R.rotation;
		float[] ref_normal 		= refPoly.normals[index];

		float[][] incMatrix 	= I.rotation;
		float[] incPosition 	= I.position;
		float[][] incTranspose 	= Matrix2.transpose(incMatrix);
		
		// transform normal from model-space A to B
		ref_normal = Matrix2.transform(refMatrix, ref_normal);
		ref_normal = Matrix2.transform(incTranspose, ref_normal);
		
		// find most anti-normal face on incident polygon
		int incident_face = 0;
		float min_dot = Float.MAX_VALUE;
		
		float[][] vertices 	= incPoly.vertices;
		float[][] normals 	= incPoly.normals;
		
		int l = vertices.length;
		for (int i=0; i<l; i++)
		{
			float[] norm = normals[i];
			
			float dot = Vector2.dot(ref_normal, norm);
			if (dot < min_dot)
			{
				min_dot = dot;
				incident_face = i;
			}
		}
		
		v[0] = Matrix2.transform(incMatrix, vertices[incident_face] );
		v[0] = Vector2.add(v[0], incPosition);
		
		incident_face = (incident_face+1)%l;
		
		v[1] = Matrix2.transform(incMatrix, vertices[incident_face] );
		v[1] = Vector2.add(v[1], incPosition);
	}
	
	/*
	 * 
	 */
	private int clip(float[] n, float c, float[][] face)
	{
		int sp = 0;
		float[][] out = 
		{
			Vector2.copy(face[0]), 
			Vector2.copy(face[1])
		};
		
		float d1 = Vector2.dot(n, face[0]) - c;
		float d2 = Vector2.dot(n, face[1]) - c;

		if (d1 <= 0.0f) out[sp++] = Vector2.copy(face[0]);
		if (d2 <= 0.0f) out[sp++] = Vector2.copy(face[1]);
		
		// xor, if one is negative
		if (d1*d2 < 0.0f)
		{
			float lambda = d1 / (d1 - d2);
			float[] A = Vector2.sub(face[1], face[0]);
			A = Vector2.mul(A, lambda);
			out[sp++] = Vector2.add(A ,face[0]);
		}
		face[0] = out[0];
		face[1] = out[1];
		return sp;
	}
}
