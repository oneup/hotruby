/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

package Box2D.Dynamics.Contacts{

	
import Box2D.Dynamics.*
import Box2D.Collision.*
import Box2D.Common.Math.*
import Box2D.Common.*
import Box2D.Dynamics.Contacts.*


public class b2ContactSolver
{
	public function b2ContactSolver(contacts:Array, contactCount:int, allocator:*){
		m_allocator = allocator;
		
		var i:int;
		var tVec:b2Vec2;
		var tMat:b2Mat22;
		
		m_constraintCount = 0;
		for (i = 0; i < contactCount; ++i)
		{
			m_constraintCount += contacts[i].GetManifoldCount();
		}
		
		// fill array
		for (i = 0; i < m_constraintCount; i++){
			m_constraints[i] = new b2ContactConstraint();
		}
		
		var count:int = 0;
		for (i = 0; i < contactCount; ++i)
		{
			var contact:b2Contact = contacts[i];
			var b1:b2Body = contact.m_shape1.m_body;
			var b2:b2Body = contact.m_shape2.m_body;
			var manifoldCount:int = contact.GetManifoldCount();
			var manifolds:Array = contact.GetManifolds();
			var friction:Number = contact.m_friction;
			var restitution:Number = contact.m_restitution;
			
			//var v1:b2Vec2 = b1.m_linearVelocity.Copy();
			var v1X:Number = b1.m_linearVelocity.x;
			var v1Y:Number = b1.m_linearVelocity.y;
			//var v2:b2Vec2 = b2.m_linearVelocity.Copy();
			var v2X:Number = b2.m_linearVelocity.x;
			var v2Y:Number = b2.m_linearVelocity.y;
			var w1:Number = b1.m_angularVelocity;
			var w2:Number = b2.m_angularVelocity;
			
			for (var j:int = 0; j < manifoldCount; ++j)
			{
				var manifold:b2Manifold = manifolds[ j ];
				
				//b2Settings.b2Assert(manifold.pointCount > 0);
				
				//var normal:b2Vec2 = manifold.normal.Copy();
				var normalX:Number = manifold.normal.x;
				var normalY:Number = manifold.normal.y;
				
				//b2Settings.b2Assert(count < m_constraintCount);
				var c:b2ContactConstraint = m_constraints[ count ];
				c.body1 = b1; //p
				c.body2 = b2; //p
				c.manifold = manifold; //p
				//c.normal = normal;
				c.normal.x = normalX;
				c.normal.y = normalY;
				c.pointCount = manifold.pointCount;
				c.friction = friction;
				c.restitution = restitution;
				
				for (var k:uint = 0; k < c.pointCount; ++k)
				{
					var cp:b2ContactPoint = manifold.points[ k ];
					var ccp:b2ContactConstraintPoint = c.points[ k ];
					
					ccp.normalImpulse = cp.normalImpulse;
					ccp.tangentImpulse = cp.tangentImpulse;
					ccp.separation = cp.separation;
					
					//var r1:b2Vec2 = b2Math.SubtractVV( cp.position, b1.m_position );
					var r1X:Number = cp.position.x - b1.m_position.x;
					var r1Y:Number = cp.position.y - b1.m_position.y;
					//var r2:b2Vec2 = b2Math.SubtractVV( cp.position, b2.m_position );
					var r2X:Number = cp.position.x - b2.m_position.x;
					var r2Y:Number = cp.position.y - b2.m_position.y;
					
					//ccp.localAnchor1 = b2Math.b2MulTMV(b1.m_R, r1);
					tVec = ccp.localAnchor1;
					tMat = b1.m_R;
					tVec.x = r1X * tMat.col1.x + r1Y * tMat.col1.y;//b2Math.b2Dot(this, A.col1);
					tVec.y = r1X * tMat.col2.x + r1Y * tMat.col2.y;//b2Math.b2Dot(this, A.col2);
					
					//ccp.localAnchor2 = b2Math.b2MulTMV(b2.m_R, r2);
					tVec = ccp.localAnchor2;
					tMat = b2.m_R;
					tVec.x = r2X * tMat.col1.x + r2Y * tMat.col1.y;
					tVec.y = r2X * tMat.col2.x + r2Y * tMat.col2.y;
					
					var r1Sqr:Number = r1X * r1X + r1Y * r1Y;//b2Math.b2Dot(r1, r1);
					var r2Sqr:Number = r2X * r2X + r2Y * r2Y;//b2Math.b2Dot(r2, r2);
					
					//var rn1:Number = b2Math.b2Dot(r1, normal);
					var rn1:Number = r1X*normalX + r1Y*normalY;
					//var rn2:Number = b2Math.b2Dot(r2, normal);
					var rn2:Number = r2X*normalX + r2Y*normalY;
					var kNormal:Number = b1.m_invMass + b2.m_invMass;
					kNormal += b1.m_invI * (r1Sqr - rn1 * rn1) + b2.m_invI * (r2Sqr - rn2 * rn2);
					//b2Settings.b2Assert(kNormal > Number.MIN_VALUE);
					ccp.normalMass = 1.0 / kNormal;
					
					//var tangent:b2Vec2 = b2Math.b2CrossVF(normal, 1.0);
					var tangentX:Number = normalY
					var tangentY:Number = -normalX;
					
					//var rt1:Number = b2Math.b2Dot(r1, tangent);
					var rt1:Number = r1X*tangentX + r1Y*tangentY;
					//var rt2:Number = b2Math.b2Dot(r2, tangent);
					var rt2:Number = r2X*tangentX + r2Y*tangentY;
					var kTangent:Number = b1.m_invMass + b2.m_invMass;
					kTangent += b1.m_invI * (r1Sqr - rt1 * rt1) + b2.m_invI * (r2Sqr - rt2 * rt2);
					//b2Settings.b2Assert(kTangent > Number.MIN_VALUE);
					ccp.tangentMass = 1.0 /  kTangent;
					
					// Setup a velocity bias for restitution.
					ccp.velocityBias = 0.0;
					if (ccp.separation > 0.0)
					{
						ccp.velocityBias = -60.0 * ccp.separation; // TODO_ERIN b2TimeStep
					}
					//var vRel:Number = b2Math.b2Dot(c.normal, b2Math.SubtractVV( b2Math.SubtractVV( b2Math.AddVV( v2, b2Math.b2CrossFV(w2, r2)), v1 ), b2Math.b2CrossFV(w1, r1)));
					var tX:Number = v2X + (-w2*r2Y) - v1X - (-w1*r1Y);
					var tY:Number = v2Y + (w2*r2X) - v1Y - (w1*r1X);
					//var vRel:Number = b2Dot(c.normal, tX/Y);
					var vRel:Number = c.normal.x*tX + c.normal.y*tY;
					if (vRel < -b2Settings.b2_velocityThreshold)
					{
						ccp.velocityBias += -c.restitution * vRel;
					}
				}
				
				++count;
			}
		}
		
		//b2Settings.b2Assert(count == m_constraintCount);
	}
	//~b2ContactSolver();

	public function PreSolve() : void{
		var tVec:b2Vec2;
		var tVec2:b2Vec2;
		var tMat:b2Mat22;
		
		// Warm start.
		for (var i:int = 0; i < m_constraintCount; ++i)
		{
			var c:b2ContactConstraint = m_constraints[ i ];
			
			var b1:b2Body = c.body1;
			var b2:b2Body = c.body2;
			var invMass1:Number = b1.m_invMass;
			var invI1:Number = b1.m_invI;
			var invMass2:Number = b2.m_invMass;
			var invI2:Number = b2.m_invI;
			//var normal:b2Vec2 = new b2Vec2(c.normal.x, c.normal.y);
			var normalX:Number = c.normal.x;
			var normalY:Number = c.normal.y;
			//var tangent:b2Vec2 = b2Math.b2CrossVF(normal, 1.0);
			var tangentX:Number = normalY;
			var tangentY:Number = -normalX;
			
			var j:int;
			var tCount:int;
			if (b2World.s_enableWarmStarting)
			{
				tCount = c.pointCount;
				for (j = 0; j < tCount; ++j)
				{
					var ccp:b2ContactConstraintPoint = c.points[ j ];
					//var P:b2Vec2 = b2Math.AddVV( b2Math.MulFV(ccp.normalImpulse, normal), b2Math.MulFV(ccp.tangentImpulse, tangent));
					var PX:Number = ccp.normalImpulse*normalX + ccp.tangentImpulse*tangentX;
					var PY:Number = ccp.normalImpulse*normalY + ccp.tangentImpulse*tangentY;
					
					//var r1:b2Vec2 = b2Math.b2MulMV(b1.m_R, ccp.localAnchor1);
					tMat = b1.m_R;
					tVec = ccp.localAnchor1;
					var r1X:Number = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
					var r1Y:Number = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
					
					//var r2:b2Vec2 = b2Math.b2MulMV(b2.m_R, ccp.localAnchor2);
					tMat = b2.m_R;
					tVec = ccp.localAnchor2;
					var r2X:Number = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
					var r2Y:Number = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
					
					//b1.m_angularVelocity -= invI1 * b2Math.b2CrossVV(r1, P);
					b1.m_angularVelocity -= invI1 * (r1X * PY - r1Y * PX);
					//b1.m_linearVelocity.Subtract( b2Math.MulFV(invMass1, P) );
					b1.m_linearVelocity.x -= invMass1 * PX;
					b1.m_linearVelocity.y -= invMass1 * PY;
					//b2.m_angularVelocity += invI2 * b2Math.b2CrossVV(r2, P);
					b2.m_angularVelocity += invI2 * (r2X * PY - r2Y * PX);
					//b2.m_linearVelocity.Add( b2Math.MulFV(invMass2, P) );
					b2.m_linearVelocity.x += invMass2 * PX;
					b2.m_linearVelocity.y += invMass2 * PY;
					
					ccp.positionImpulse = 0.0;
				}
			}
			else{
				tCount = c.pointCount;
				for (j = 0; j < tCount; ++j)
				{
					var ccp2:b2ContactConstraintPoint = c.points[ j ];
					ccp2.normalImpulse = 0.0;
					ccp2.tangentImpulse = 0.0;
					
					ccp2.positionImpulse = 0.0;
				}
			}
		}
	}
	public function SolveVelocityConstraints() : void{
		var j:int;
		var ccp:b2ContactConstraintPoint;
		var r1X:Number;
		var r1Y:Number;
		var r2X:Number;
		var r2Y:Number;
		var dvX:Number;
		var dvY:Number;
		var lambda:Number;
		var newImpulse:Number;
		var PX:Number;
		var PY:Number;
		
		var tMat:b2Mat22;
		var tVec:b2Vec2;
		
		for (var i:int = 0; i < m_constraintCount; ++i)
		{
			var c:b2ContactConstraint = m_constraints[ i ];
			var b1:b2Body = c.body1;
			var b2:b2Body = c.body2;
			var b1_angularVelocity:Number = b1.m_angularVelocity;
			var b1_linearVelocity:b2Vec2 = b1.m_linearVelocity;
			var b2_angularVelocity:Number = b2.m_angularVelocity;
			var b2_linearVelocity:b2Vec2 = b2.m_linearVelocity;
			
			var invMass1:Number = b1.m_invMass;
			var invI1:Number = b1.m_invI;
			var invMass2:Number = b2.m_invMass;
			var invI2:Number = b2.m_invI;
			//var normal:b2Vec2 = new b2Vec2(c.normal.x, c.normal.y);
			var normalX:Number = c.normal.x;
			var normalY:Number = c.normal.y;
			//var tangent:b2Vec2 = b2Math.b2CrossVF(normal, 1.0);
			var tangentX:Number = normalY;
			var tangentY:Number = -normalX;
			
			// Solver normal constraints
			var tCount:int = c.pointCount;
			for (j = 0; j < tCount; ++j)
			{
				ccp = c.points[ j ];
				
				//r1 = b2Math.b2MulMV(b1.m_R, ccp.localAnchor1);
				tMat = b1.m_R;
				tVec = ccp.localAnchor1;
				r1X = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
				r1Y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
				//r2 = b2Math.b2MulMV(b2.m_R, ccp.localAnchor2);
				tMat = b2.m_R;
				tVec = ccp.localAnchor2;
				r2X = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
				r2Y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
				
				// Relative velocity at contact
				//var dv:b2Vec2 = b2Math.SubtractVV( b2Math.AddVV( b2.m_linearVelocity, b2Math.b2CrossFV(b2.m_angularVelocity, r2)), b2Math.SubtractVV(b1.m_linearVelocity, b2Math.b2CrossFV(b1.m_angularVelocity, r1)));
				//dv = b2Math.SubtractVV(b2Math.SubtractVV( b2Math.AddVV( b2.m_linearVelocity, b2Math.b2CrossFV(b2.m_angularVelocity, r2)), b1.m_linearVelocity), b2Math.b2CrossFV(b1.m_angularVelocity, r1));
				dvX = b2_linearVelocity.x + (-b2_angularVelocity * r2Y) - b1_linearVelocity.x - (-b1_angularVelocity * r1Y);
				dvY = b2_linearVelocity.y + (b2_angularVelocity * r2X) - b1_linearVelocity.y - (b1_angularVelocity * r1X);
				
				// Compute normal impulse
				//var vn:Number = b2Math.b2Dot(dv, normal);
				var vn:Number = dvX * normalX + dvY * normalY;
				lambda = -ccp.normalMass * (vn - ccp.velocityBias);
				
				// b2Clamp the accumulated impulse
				newImpulse = b2Math.b2Max(ccp.normalImpulse + lambda, 0.0);
				lambda = newImpulse - ccp.normalImpulse;
				
				// Apply contact impulse
				//P = b2Math.MulFV(lambda, normal);
				PX = lambda * normalX;
				PY = lambda * normalY;
				
				//b1.m_linearVelocity.Subtract( b2Math.MulFV( invMass1, P ) );
				b1_linearVelocity.x -= invMass1 * PX;
				b1_linearVelocity.y -= invMass1 * PY;
				b1_angularVelocity -= invI1 * (r1X * PY - r1Y * PX);//invI1 * b2Math.b2CrossVV(r1, P);
				
				//b2.m_linearVelocity.Add( b2Math.MulFV( invMass2, P ) );
				b2_linearVelocity.x += invMass2 * PX;
				b2_linearVelocity.y += invMass2 * PY;
				b2_angularVelocity += invI2 * (r2X * PY - r2Y * PX);//invI2 * b2Math.b2CrossVV(r2, P);
				
				ccp.normalImpulse = newImpulse;
				
				
				
				// MOVED FROM BELOW
				// Relative velocity at contact
				//var dv:b2Vec2 = b2.m_linearVelocity + b2Cross(b2.m_angularVelocity, r2) - b1.m_linearVelocity - b2Cross(b1.m_angularVelocity, r1);
				//dv =  b2Math.SubtractVV(b2Math.SubtractVV(b2Math.AddVV(b2.m_linearVelocity, b2Math.b2CrossFV(b2.m_angularVelocity, r2)), b1.m_linearVelocity), b2Math.b2CrossFV(b1.m_angularVelocity, r1));
				dvX = b2_linearVelocity.x + (-b2_angularVelocity * r2Y) - b1_linearVelocity.x - (-b1_angularVelocity * r1Y);
				dvY = b2_linearVelocity.y + (b2_angularVelocity * r2X) - b1_linearVelocity.y - (b1_angularVelocity * r1X);
				
				// Compute tangent impulse
				var vt:Number = dvX*tangentX + dvY*tangentY;//b2Math.b2Dot(dv, tangent);
				lambda = ccp.tangentMass * (-vt);
				
				// b2Clamp the accumulated impulse
				var maxFriction:Number = c.friction * ccp.normalImpulse;
				newImpulse = b2Math.b2Clamp(ccp.tangentImpulse + lambda, -maxFriction, maxFriction);
				lambda = newImpulse - ccp.tangentImpulse;
				
				// Apply contact impulse
				//P = b2Math.MulFV(lambda, tangent);
				PX = lambda * tangentX;
				PY = lambda * tangentY;
				
				//b1.m_linearVelocity.Subtract( b2Math.MulFV( invMass1, P ) );
				b1_linearVelocity.x -= invMass1 * PX;
				b1_linearVelocity.y -= invMass1 * PY;
				b1_angularVelocity -= invI1 * (r1X * PY - r1Y * PX);//invI1 * b2Math.b2CrossVV(r1, P);
				
				//b2.m_linearVelocity.Add( b2Math.MulFV( invMass2, P ) );
				b2_linearVelocity.x += invMass2 * PX;
				b2_linearVelocity.y += invMass2 * PY;
				b2_angularVelocity += invI2 * (r2X * PY - r2Y * PX);//invI2 * b2Math.b2CrossVV(r2, P);
				
				ccp.tangentImpulse = newImpulse;
			}
			
			
			
			// Solver tangent constraints
			// MOVED ABOVE FOR EFFICIENCY
			/*for (j = 0; j < tCount; ++j)
			{
				ccp = c.points[ j ];
				
				//r1 = b2Math.b2MulMV(b1.m_R, ccp.localAnchor1);
				tMat = b1.m_R;
				tVec = ccp.localAnchor1;
				r1X = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
				r1Y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
				//r2 = b2Math.b2MulMV(b2.m_R, ccp.localAnchor2);
				tMat = b2.m_R;
				tVec = ccp.localAnchor2;
				r2X = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
				r2Y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
				
				// Relative velocity at contact
				//var dv:b2Vec2 = b2.m_linearVelocity + b2Cross(b2.m_angularVelocity, r2) - b1.m_linearVelocity - b2Cross(b1.m_angularVelocity, r1);
				//dv =  b2Math.SubtractVV(b2Math.SubtractVV(b2Math.AddVV(b2.m_linearVelocity, b2Math.b2CrossFV(b2.m_angularVelocity, r2)), b1.m_linearVelocity), b2Math.b2CrossFV(b1.m_angularVelocity, r1));
				dvX = b2_linearVelocity.x + (-b2_angularVelocity * r2Y) - b1_linearVelocity.x - (-b1_angularVelocity * r1Y);
				dvY = b2_linearVelocity.y + (b2_angularVelocity * r2X) - b1_linearVelocity.y - (b1_angularVelocity * r1X);
				
				// Compute tangent impulse
				var vt:Number = dvX*tangentX + dvY*tangentY;//b2Math.b2Dot(dv, tangent);
				lambda = ccp.tangentMass * (-vt);
				
				// b2Clamp the accumulated impulse
				var maxFriction:Number = c.friction * ccp.normalImpulse;
				newImpulse = b2Math.b2Clamp(ccp.tangentImpulse + lambda, -maxFriction, maxFriction);
				lambda = newImpulse - ccp.tangentImpulse;
				
				// Apply contact impulse
				//P = b2Math.MulFV(lambda, tangent);
				PX = lambda * tangentX;
				PY = lambda * tangentY;
				
				//b1.m_linearVelocity.Subtract( b2Math.MulFV( invMass1, P ) );
				b1_linearVelocity.x -= invMass1 * PX;
				b1_linearVelocity.y -= invMass1 * PY;
				b1_angularVelocity -= invI1 * (r1X * PY - r1Y * PX);//invI1 * b2Math.b2CrossVV(r1, P);
				
				//b2.m_linearVelocity.Add( b2Math.MulFV( invMass2, P ) );
				b2_linearVelocity.x += invMass2 * PX;
				b2_linearVelocity.y += invMass2 * PY;
				b2_angularVelocity += invI2 * (r2X * PY - r2Y * PX);//invI2 * b2Math.b2CrossVV(r2, P);
				
				ccp.tangentImpulse = newImpulse;
			}*/
			
			// Update angular velocity
			b1.m_angularVelocity = b1_angularVelocity;
			b2.m_angularVelocity = b2_angularVelocity;
		}
	}
	public function SolvePositionConstraints(beta:Number):Boolean{
		var minSeparation:Number = 0.0;
		
		var tMat:b2Mat22;
		var tVec:b2Vec2;
		
		for (var i:int = 0; i < m_constraintCount; ++i)
		{
			var c:b2ContactConstraint = m_constraints[ i ];
			var b1:b2Body = c.body1;
			var b2:b2Body = c.body2;
			var b1_position:b2Vec2 = b1.m_position;
			var b1_rotation:Number = b1.m_rotation;
			var b2_position:b2Vec2 = b2.m_position;
			var b2_rotation:Number = b2.m_rotation;
			
			var invMass1:Number = b1.m_invMass;
			var invI1:Number = b1.m_invI;
			var invMass2:Number = b2.m_invMass;
			var invI2:Number = b2.m_invI;
			//var normal:b2Vec2 = new b2Vec2(c.normal.x, c.normal.y);
			var normalX:Number = c.normal.x;
			var normalY:Number = c.normal.y;
			//var tangent:b2Vec2 = b2Math.b2CrossVF(normal, 1.0);
			var tangentX:Number = normalY;
			var tangentY:Number = -normalX;
			
			// Solver normal constraints
			var tCount:int = c.pointCount;
			for (var j:int = 0; j < tCount; ++j)
			{
				var ccp:b2ContactConstraintPoint = c.points[ j ];
				
				//r1 = b2Math.b2MulMV(b1.m_R, ccp.localAnchor1);
				tMat = b1.m_R;
				tVec = ccp.localAnchor1;
				var r1X:Number = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
				var r1Y:Number = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
				//r2 = b2Math.b2MulMV(b2.m_R, ccp.localAnchor2);
				tMat = b2.m_R;
				tVec = ccp.localAnchor2;
				var r2X:Number = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y
				var r2Y:Number = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y
				
				//var p1:b2Vec2 = b2Math.AddVV(b1.m_position, r1);
				var p1X:Number = b1_position.x + r1X;
				var p1Y:Number = b1_position.y + r1Y;
				
				//var p2:b2Vec2 = b2Math.AddVV(b2.m_position, r2);
				var p2X:Number = b2_position.x + r2X;
				var p2Y:Number = b2_position.y + r2Y;
				
				//var dp:b2Vec2 = b2Math.SubtractVV(p2, p1);
				var dpX:Number = p2X - p1X;
				var dpY:Number = p2Y - p1Y;
				
				// Approximate the current separation.
				//var separation:Number = b2Math.b2Dot(dp, normal) + ccp.separation;
				var separation:Number = (dpX*normalX + dpY*normalY) + ccp.separation;
				
				// Track max constraint error.
				minSeparation = b2Math.b2Min(minSeparation, separation);
				
				// Prevent large corrections and allow slop.
				var C:Number = beta * b2Math.b2Clamp(separation + b2Settings.b2_linearSlop, -b2Settings.b2_maxLinearCorrection, 0.0);
				
				// Compute normal impulse
				var dImpulse:Number = -ccp.normalMass * C;
				
				// b2Clamp the accumulated impulse
				var impulse0:Number = ccp.positionImpulse;
				ccp.positionImpulse = b2Math.b2Max(impulse0 + dImpulse, 0.0);
				dImpulse = ccp.positionImpulse - impulse0;
				
				//var impulse:b2Vec2 = b2Math.MulFV( dImpulse, normal );
				var impulseX:Number = dImpulse * normalX;
				var impulseY:Number = dImpulse * normalY;
				
				//b1.m_position.Subtract( b2Math.MulFV( invMass1, impulse ) );
				b1_position.x -= invMass1 * impulseX;
				b1_position.y -= invMass1 * impulseY;
				b1_rotation -= invI1 * (r1X * impulseY - r1Y * impulseX);//b2Math.b2CrossVV(r1, impulse);
				b1.m_R.Set(b1_rotation);
				
				//b2.m_position.Add( b2Math.MulFV( invMass2, impulse ) );
				b2_position.x += invMass2 * impulseX;
				b2_position.y += invMass2 * impulseY;
				b2_rotation += invI2 * (r2X * impulseY - r2Y * impulseX);//b2Math.b2CrossVV(r2, impulse);
				b2.m_R.Set(b2_rotation);
			}
			// Update body rotations
			b1.m_rotation = b1_rotation;
			b2.m_rotation = b2_rotation;
		}
		
		return minSeparation >= -b2Settings.b2_linearSlop;
	}
	public function PostSolve() : void{
		for (var i:int = 0; i < m_constraintCount; ++i)
		{
			var c:b2ContactConstraint = m_constraints[ i ];
			var m:b2Manifold = c.manifold;
			
			for (var j:int = 0; j < c.pointCount; ++j)
			{
				var mPoint:b2ContactPoint = m.points[j];
				var cPoint:b2ContactConstraintPoint = c.points[j];
				mPoint.normalImpulse = cPoint.normalImpulse;
				mPoint.tangentImpulse = cPoint.tangentImpulse;
			}
		}
	}

	public var m_allocator:*;
	public var m_constraints:Array = new Array();
	public var m_constraintCount:int;
};

}