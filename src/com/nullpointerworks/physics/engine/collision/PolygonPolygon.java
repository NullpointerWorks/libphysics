package com.nullpointerworks.physics.engine.collision;

import static com.nullpointerworks.physics.engine.ImpulseMath.gt;
import static com.nullpointerworks.physics.engine.MatrixMath.rotation;
import static com.nullpointerworks.physics.engine.MatrixMath.transform;
import static com.nullpointerworks.physics.engine.MatrixMath.transpose;
import static com.nullpointerworks.physics.engine.VectorMath.add;
import static com.nullpointerworks.physics.engine.VectorMath.copy;
import static com.nullpointerworks.physics.engine.VectorMath.dot;
import static com.nullpointerworks.physics.engine.VectorMath.mul;
import static com.nullpointerworks.physics.engine.VectorMath.neg;
import static com.nullpointerworks.physics.engine.VectorMath.normal;
import static com.nullpointerworks.physics.engine.VectorMath.normalize;
import static com.nullpointerworks.physics.engine.VectorMath.sub;

import com.nullpointerworks.physics.engine.CollisionSolver;
import com.nullpointerworks.physics.engine.Composite;
import com.nullpointerworks.physics.engine.manifold.Manifold;
import com.nullpointerworks.physics.engine.shape.Polygon;

public class PolygonPolygon implements CollisionSolver 
{
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
		if (gt(penetrationA, penetrationB))
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
		Polygon shapeR 	= (Polygon)R.getShape();
		float[] Rposition = R.getLinearMotion().getPosition();
		float[][] incFace = new float[2][];
		IncidentFace(incFace, R, I, reference_index);
		
		float[][] refVertices	= shapeR.vertices;
		float[][] refMatrix 	= rotation( R.getAngularMotion().getOrientation() );
		int l = refVertices.length;
		
		// get vertices from the reference polygon
		float[] v1 = refVertices[ reference_index ];
		float[] v2 = refVertices[(reference_index+1)%l ];
		
		// translate to world space
		v1 = transform(refMatrix, v1);
		v2 = transform(refMatrix, v2);
		v1 = add(v1, Rposition);
		v2 = add(v2, Rposition);
		
		// get normal and tangent of the world-space face
		float[] plane_normal 	= sub(v2, v1);
		plane_normal 			= normalize(plane_normal);
		float[] face_normal 	= normal(plane_normal, -1f);
		
		// find collision normal
		float refC 		= dot(face_normal, v1);
		float neg_side 	= -dot(plane_normal, v1);
		float pos_side 	= dot(plane_normal, v2);
		
		if ( clip(neg(plane_normal), neg_side, incFace ) < 2 ) 
			return;
		
		if ( clip(plane_normal, pos_side, incFace ) < 2 ) 
			return;
		
		// flip collision direction normal
		m.normal = face_normal;
		if (handedness)
		{
			m.normal = neg(m.normal);
		}
		
		// find separation
		int cp = 0;
		float separation1 = dot(face_normal, incFace[0]) - refC;
		float separation2 = dot(face_normal, incFace[1]) - refC;
		
		m.penetration = 0.0f;
		if (separation1 <= 0.0f)
		{
			m.contacts[cp] = copy(incFace[0]);
			m.penetration += -separation1;
			cp++;
		}
		
		if (separation2 <= 0.0f)
		{
			m.contacts[cp] = copy(incFace[1]);
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
		Polygon shapeA 	= (Polygon)A.getShape();
		Polygon shapeB 	= (Polygon)B.getShape();

		float[] Aposition = A.getLinearMotion().getPosition();
		float[] Bposition = B.getLinearMotion().getPosition();
		
		float best_dist = -Float.MAX_VALUE;
		int best_index 	= 0;

		float[][] verticesA	= shapeA.vertices;
		float[][] verticesB	= shapeB.vertices;
		float[][] normals 	= shapeA.normals;
		
		float[][] rotationA = rotation( A.getAngularMotion().getOrientation() );
		float[][] rotationB = rotation( B.getAngularMotion().getOrientation() );
		float[][] trnsposeB = transpose(rotationB);

		float[] posA = Aposition;
		float[] posB = Bposition;
		
		for (int i=0,l=verticesA.length; i<l; i++)
		{
			float[] vertex = verticesA[i];
			float[] normal = normals[i];
			
			// move normal to polygon B's model space
			float[] nw = transform(rotationA, normal);
			float[] n = transform(trnsposeB, nw);
			
			// move vertex from model space A to model space B
			float[] v = transform(rotationA, vertex);
			v = add(v, posA);
			v = sub(v, posB);
			v = transform(trnsposeB, v);
			
			// get support
			float[] s = support(verticesB, neg(n) );
			float dist = dot(n, sub(s, v) );
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
			float proj = dot(v, dir);
			
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
		Polygon refPoly = (Polygon)R.getShape();
		Polygon incPoly = (Polygon)I.getShape();
		
		float[][] refMatrix 	= rotation( R.getAngularMotion().getOrientation() );
		float[] ref_normal 		= refPoly.normals[index];
		
		float[][] incMatrix 	= rotation( I.getAngularMotion().getOrientation() );
		float[] incPosition 	= I.getLinearMotion().getPosition();
		float[][] incTranspose 	= transpose(incMatrix);
		
		// transform normal from model-space A to B
		ref_normal = transform(refMatrix, ref_normal);
		ref_normal = transform(incTranspose, ref_normal);
		
		// find most anti-normal face on incident polygon
		int incident_face = 0;
		float min_dot = Float.MAX_VALUE;
		
		float[][] vertices 	= incPoly.vertices;
		float[][] normals 	= incPoly.normals;
		
		int l = vertices.length;
		for (int i=0; i<l; i++)
		{
			float[] norm = normals[i];
			
			float dot = dot(ref_normal, norm);
			if (dot < min_dot)
			{
				min_dot = dot;
				incident_face = i;
			}
		}
		
		v[0] = transform(incMatrix, vertices[incident_face] );
		v[0] = add(v[0], incPosition);
		
		incident_face = (incident_face+1)%l;
		
		v[1] = transform(incMatrix, vertices[incident_face] );
		v[1] = add(v[1], incPosition);
	}
	
	/*
	 * 
	 */
	private int clip(float[] n, float c, float[][] face)
	{
		int sp = 0;
		float[][] out = 
		{
			copy(face[0]), 
			copy(face[1])
		};
		
		float d1 = dot(n, face[0]) - c;
		float d2 = dot(n, face[1]) - c;

		if (d1 <= 0.0f) out[sp++] = copy(face[0]);
		if (d2 <= 0.0f) out[sp++] = copy(face[1]);
		
		// xor, if one is negative
		if (d1*d2 < 0.0f)
		{
			float lambda = d1 / (d1 - d2);
			float[] A = sub(face[1], face[0]);
			A = mul(A, lambda);
			out[sp++] = add(A ,face[0]);
		}
		face[0] = out[0];
		face[1] = out[1];
		return sp;
	}
}
