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

package Box2D.Dynamics.Joints{
	
	
	
import Box2D.Common.Math.*
import Box2D.Common.*
import Box2D.Dynamics.*
	
	
	
public class b2PulleyJoint extends b2Joint
{
	public override function GetAnchor1():b2Vec2{
		//return m_body1->m_position + b2Mul(m_body1->m_R, m_localAnchor1);
		var tMat:b2Mat22 = m_body1.m_R;
		return new b2Vec2(	m_body1.m_position.x + (tMat.col1.x * m_localAnchor1.x + tMat.col2.x * m_localAnchor1.y), 
							m_body1.m_position.y + (tMat.col1.y * m_localAnchor1.x + tMat.col2.y * m_localAnchor1.y));
	}
	public override function GetAnchor2():b2Vec2{
		//return m_body2->m_position + b2Mul(m_body2->m_R, m_localAnchor2);
		var tMat:b2Mat22 = m_body2.m_R;
		return new b2Vec2(	m_body2.m_position.x + (tMat.col1.x * m_localAnchor2.x + tMat.col2.x * m_localAnchor2.y), 
							m_body2.m_position.y + (tMat.col1.y * m_localAnchor2.x + tMat.col2.y * m_localAnchor2.y));
	}

	public function GetGroundPoint1():b2Vec2{
		//return m_ground->m_position + m_groundAnchor1;
		return new b2Vec2(m_ground.m_position.x + m_groundAnchor1.x, m_ground.m_position.y + m_groundAnchor1.y);
	}
	public function GetGroundPoint2():b2Vec2{
		return new b2Vec2(m_ground.m_position.x + m_groundAnchor2.x, m_ground.m_position.y + m_groundAnchor2.y);
	}

	public override function GetReactionForce(invTimeStep:Number):b2Vec2{
		//b2Vec2 F(0.0f, 0.0f); // = (m_pulleyImpulse * invTimeStep) * m_u;
		return new b2Vec2();
	}
	public override function GetReactionTorque(invTimeStep:Number):Number{
		return 0.0;
	}

	public function GetLength1():Number{
		var tMat:b2Mat22;
		//b2Vec2 p = m_body1->m_position + b2Mul(m_body1->m_R, m_localAnchor1);
		tMat = m_body1.m_R;
		var pX:Number = m_body1.m_position.x + (tMat.col1.x * m_localAnchor1.x + tMat.col2.x * m_localAnchor1.y);
		var pY:Number = m_body1.m_position.y + (tMat.col1.y * m_localAnchor1.x + tMat.col2.y * m_localAnchor1.y);
		//b2Vec2 s = m_ground->m_position + m_groundAnchor1;
		//b2Vec2 d = p - s;
		var dX:Number = pX - (m_ground.m_position.x + m_groundAnchor1.x);
		var dY:Number = pY - (m_ground.m_position.y + m_groundAnchor1.y);
		return Math.sqrt(dX*dX + dY*dY);
	}
	public function GetLength2():Number{
		var tMat:b2Mat22;
		//b2Vec2 p = m_body2->m_position + b2Mul(m_body2->m_R, m_localAnchor2);
		tMat = m_body2.m_R;
		var pX:Number = m_body2.m_position.x + (tMat.col1.x * m_localAnchor2.x + tMat.col2.x * m_localAnchor2.y);
		var pY:Number = m_body2.m_position.y + (tMat.col1.y * m_localAnchor2.x + tMat.col2.y * m_localAnchor2.y);
		//b2Vec2 s = m_ground->m_position + m_groundAnchor2;
		//b2Vec2 d = p - s;
		var dX:Number = pX - (m_ground.m_position.x + m_groundAnchor2.x);
		var dY:Number = pY - (m_ground.m_position.y + m_groundAnchor2.y);
		return Math.sqrt(dX*dX + dY*dY);
	}

	public function GetRatio():Number{
		return m_ratio;
	}

	//--------------- Internals Below -------------------

	public function b2PulleyJoint(def:b2PulleyJointDef){
		
		// parent
		super(def);
		
		var tMat:b2Mat22;
		var tX:Number;
		var tY:Number;
		
		m_ground = m_body1.m_world.m_groundBody;
		//m_groundAnchor1 = def.groundPoint1 - m_ground.m_position;
		m_groundAnchor1.x = def.groundPoint1.x - m_ground.m_position.x;
		m_groundAnchor1.y = def.groundPoint1.y - m_ground.m_position.y;
		//m_groundAnchor2 = def.groundPoint2 - m_ground.m_position;
		m_groundAnchor2.x = def.groundPoint2.x - m_ground.m_position.x;
		m_groundAnchor2.y = def.groundPoint2.y - m_ground.m_position.y;
		//m_localAnchor1 = b2MulT(m_body1.m_R, def.anchorPoint1 - m_body1.m_position);
		tMat = m_body1.m_R;
		tX = def.anchorPoint1.x - m_body1.m_position.x;
		tY = def.anchorPoint1.y - m_body1.m_position.y;
		m_localAnchor1.x = tX*tMat.col1.x + tY*tMat.col1.y;
		m_localAnchor1.y = tX*tMat.col2.x + tY*tMat.col2.y;
		//m_localAnchor2 = b2MulT(m_body2.m_R, def.anchorPoint2 - m_body2.m_position);
		tMat = m_body2.m_R;
		tX = def.anchorPoint2.x - m_body2.m_position.x;
		tY = def.anchorPoint2.y - m_body2.m_position.y;
		m_localAnchor2.x = tX*tMat.col1.x + tY*tMat.col1.y;
		m_localAnchor2.y = tX*tMat.col2.x + tY*tMat.col2.y;
		
		m_ratio = def.ratio;
		
		//var d1:b2Vec2 = def.groundPoint1 - def.anchorPoint1;
		tX = def.groundPoint1.x - def.anchorPoint1.x;
		tY = def.groundPoint1.y - def.anchorPoint1.y;
		var d1Len:Number = Math.sqrt(tX*tX + tY*tY);
		//var d2:b2Vec2 = def.groundPoint2 - def.anchorPoint2;
		tX = def.groundPoint2.x - def.anchorPoint2.x;
		tY = def.groundPoint2.y - def.anchorPoint2.y;
		var d2Len:Number = Math.sqrt(tX*tX + tY*tY);
		
		var length1:Number = b2Math.b2Max(0.5 * b2_minPulleyLength, d1Len);
		var length2:Number = b2Math.b2Max(0.5 * b2_minPulleyLength, d2Len);
		
		m_constant = length1 + m_ratio * length2;
		
		m_maxLength1 = b2Math.b2Clamp(def.maxLength1, length1, m_constant - m_ratio * b2_minPulleyLength);
		m_maxLength2 = b2Math.b2Clamp(def.maxLength2, length2, (m_constant - b2_minPulleyLength) / m_ratio);
		
		m_pulleyImpulse = 0.0;
		m_limitImpulse1 = 0.0;
		m_limitImpulse2 = 0.0;
		
	}

	public override function PrepareVelocitySolver() : void{
		var b1:b2Body = m_body1;
		var b2:b2Body = m_body2;
		
		var tMat:b2Mat22;
		
		//b2Vec2 r1 = b2Mul(b1->m_R, m_localAnchor1);
		tMat = b1.m_R;
		var r1X:Number = tMat.col1.x * m_localAnchor1.x + tMat.col2.x * m_localAnchor1.y;
		var r1Y:Number = tMat.col1.y * m_localAnchor1.x + tMat.col2.y * m_localAnchor1.y;
		//b2Vec2 r2 = b2Mul(b2->m_R, m_localAnchor2);
		tMat = b2.m_R;
		var r2X:Number = tMat.col1.x * m_localAnchor2.x + tMat.col2.x * m_localAnchor2.y;
		var r2Y:Number = tMat.col1.y * m_localAnchor2.x + tMat.col2.y * m_localAnchor2.y;
		
		//b2Vec2 p1 = b1->m_position + r1;
		var p1X:Number = b1.m_position.x + r1X;
		var p1Y:Number = b1.m_position.y + r1Y;
		//b2Vec2 p2 = b2->m_position + r2;
		var p2X:Number = b2.m_position.x + r2X;
		var p2Y:Number = b2.m_position.y + r2Y;
		
		//b2Vec2 s1 = m_ground->m_position + m_groundAnchor1;
		var s1X:Number = m_ground.m_position.x + m_groundAnchor1.x;
		var s1Y:Number = m_ground.m_position.y + m_groundAnchor1.y;
		//b2Vec2 s2 = m_ground->m_position + m_groundAnchor2;
		var s2X:Number = m_ground.m_position.x + m_groundAnchor2.x;
		var s2Y:Number = m_ground.m_position.y + m_groundAnchor2.y;
		
		// Get the pulley axes.
		//m_u1 = p1 - s1;
		m_u1.Set(p1X - s1X, p1Y - s1Y);
		//m_u2 = p2 - s2;
		m_u2.Set(p2X - s2X, p2Y - s2Y);
		
		var length1:Number = m_u1.Length();
		var length2:Number = m_u2.Length();
		
		if (length1 > b2Settings.b2_linearSlop)
		{
			//m_u1 *= 1.0f / length1;
			m_u1.Multiply(1.0 / length1);
		}
		else
		{
			m_u1.SetZero();
		}
		
		if (length2 > b2Settings.b2_linearSlop)
		{
			//m_u2 *= 1.0f / length2;
			m_u2.Multiply(1.0 / length2);
		}
		else
		{
			m_u2.SetZero();
		}
		
		if (length1 < m_maxLength1)
		{
			m_limitState1 = e_inactiveLimit;
			m_limitImpulse1 = 0.0;
		}
		else
		{
			m_limitState1 = e_atUpperLimit;
			m_limitPositionImpulse1 = 0.0;
		}
		
		if (length2 < m_maxLength2)
		{
			m_limitState2 = e_inactiveLimit;
			m_limitImpulse2 = 0.0;
		}
		else
		{
			m_limitState2 = e_atUpperLimit;
			m_limitPositionImpulse2 = 0.0;
		}
		
		// Compute effective mass.
		//var cr1u1:Number = b2Cross(r1, m_u1);
		var cr1u1:Number = r1X * m_u1.y - r1Y * m_u1.x;
		//var cr2u2:Number = b2Cross(r2, m_u2);
		var cr2u2:Number = r2X * m_u2.y - r2Y * m_u2.x;
		
		m_limitMass1 = b1.m_invMass + b1.m_invI * cr1u1 * cr1u1;
		m_limitMass2 = b2.m_invMass + b2.m_invI * cr2u2 * cr2u2;
		m_pulleyMass = m_limitMass1 + m_ratio * m_ratio * m_limitMass2;
		//b2Settings.b2Assert(m_limitMass1 > Number.MIN_VALUE);
		//b2Settings.b2Assert(m_limitMass2 > Number.MIN_VALUE);
		//b2Settings.b2Assert(m_pulleyMass > Number.MIN_VALUE);
		m_limitMass1 = 1.0 / m_limitMass1;
		m_limitMass2 = 1.0 / m_limitMass2;
		m_pulleyMass = 1.0 / m_pulleyMass;
		
		// Warm starting.
		//b2Vec2 P1 = (-m_pulleyImpulse - m_limitImpulse1) * m_u1;
		var P1X:Number = (-m_pulleyImpulse - m_limitImpulse1) * m_u1.x;
		var P1Y:Number = (-m_pulleyImpulse - m_limitImpulse1) * m_u1.y;
		//b2Vec2 P2 = (-m_ratio * m_pulleyImpulse - m_limitImpulse2) * m_u2;
		var P2X:Number = (-m_ratio * m_pulleyImpulse - m_limitImpulse2) * m_u2.x;
		var P2Y:Number = (-m_ratio * m_pulleyImpulse - m_limitImpulse2) * m_u2.y;
		//b1.m_linearVelocity += b1.m_invMass * P1;
		b1.m_linearVelocity.x += b1.m_invMass * P1X;
		b1.m_linearVelocity.y += b1.m_invMass * P1Y;
		//b1.m_angularVelocity += b1.m_invI * b2Cross(r1, P1);
		b1.m_angularVelocity += b1.m_invI * (r1X * P1Y - r1Y * P1X);
		//b2.m_linearVelocity += b2.m_invMass * P2;
		b2.m_linearVelocity.x += b2.m_invMass * P2X;
		b2.m_linearVelocity.y += b2.m_invMass * P2Y;
		//b2.m_angularVelocity += b2.m_invI * b2Cross(r2, P2);
		b2.m_angularVelocity += b2.m_invI * (r2X * P2Y - r2Y * P2X);
	}
	
	public override function SolveVelocityConstraints(step:b2TimeStep) : void{
		var b1:b2Body = m_body1;
		var b2:b2Body = m_body2;
		
		var tMat:b2Mat22;
		
		//var r1:b2Vec2 = b2Mul(b1.m_R, m_localAnchor1);
		tMat = b1.m_R;
		var r1X:Number = tMat.col1.x * m_localAnchor1.x + tMat.col2.x * m_localAnchor1.y;
		var r1Y:Number = tMat.col1.y * m_localAnchor1.x + tMat.col2.y * m_localAnchor1.y;
		//var r2:b2Vec2 = b2Mul(b2.m_R, m_localAnchor2);
		tMat = b2.m_R;
		var r2X:Number = tMat.col1.x * m_localAnchor2.x + tMat.col2.x * m_localAnchor2.y;
		var r2Y:Number = tMat.col1.y * m_localAnchor2.x + tMat.col2.y * m_localAnchor2.y;
		
		// temp vars
		var v1X:Number;
		var v1Y:Number;
		var v2X:Number;
		var v2Y:Number;
		var P1X:Number;
		var P1Y:Number;
		var P2X:Number;
		var P2Y:Number;
		var Cdot:Number;
		var impulse:Number;
		var oldLimitImpulse:Number;
		
		//{
			//b2Vec2 v1 = b1->m_linearVelocity + b2Cross(b1->m_angularVelocity, r1);
			v1X = b1.m_linearVelocity.x + (-b1.m_angularVelocity * r1Y);
			v1Y = b1.m_linearVelocity.y + (b1.m_angularVelocity * r1X);
			//b2Vec2 v2 = b2->m_linearVelocity + b2Cross(b2->m_angularVelocity, r2);
			v2X = b2.m_linearVelocity.x + (-b2.m_angularVelocity * r2Y);
			v2Y = b2.m_linearVelocity.y + (b2.m_angularVelocity * r2X);
			
			//Cdot = -b2Dot(m_u1, v1) - m_ratio * b2Dot(m_u2, v2);
			Cdot = -(m_u1.x * v1X + m_u1.y * v1Y) - m_ratio * (m_u2.x * v2X + m_u2.y * v2Y);
			impulse = -m_pulleyMass * Cdot;
			m_pulleyImpulse += impulse;
			
			//b2Vec2 P1 = -impulse * m_u1;
			P1X = -impulse * m_u1.x;
			P1Y = -impulse * m_u1.y;
			//b2Vec2 P2 = -m_ratio * impulse * m_u2;
			P2X = -m_ratio * impulse * m_u2.x;
			P2Y = -m_ratio * impulse * m_u2.y;
			//b1.m_linearVelocity += b1.m_invMass * P1;
			b1.m_linearVelocity.x += b1.m_invMass * P1X;
			b1.m_linearVelocity.y += b1.m_invMass * P1Y;
			//b1.m_angularVelocity += b1.m_invI * b2Cross(r1, P1);
			b1.m_angularVelocity += b1.m_invI * (r1X * P1Y - r1Y * P1X);
			//b2.m_linearVelocity += b2.m_invMass * P2;
			b2.m_linearVelocity.x += b2.m_invMass * P2X;
			b2.m_linearVelocity.y += b2.m_invMass * P2Y;
			//b2.m_angularVelocity += b2.m_invI * b2Cross(r2, P2);
			b2.m_angularVelocity += b2.m_invI * (r2X * P2Y - r2Y * P2X);
		//}
		
		if (m_limitState1 == e_atUpperLimit)
		{
			//b2Vec2 v1 = b1->m_linearVelocity + b2Cross(b1->m_angularVelocity, r1);
			v1X = b1.m_linearVelocity.x + (-b1.m_angularVelocity * r1Y);
			v1Y = b1.m_linearVelocity.y + (b1.m_angularVelocity * r1X);
			
			//float32 Cdot = -b2Dot(m_u1, v1);
			Cdot = -(m_u1.x * v1X + m_u1.y * v1Y);
			impulse = -m_limitMass1 * Cdot;
			oldLimitImpulse = m_limitImpulse1;
			m_limitImpulse1 = b2Math.b2Max(0.0, m_limitImpulse1 + impulse);
			impulse = m_limitImpulse1 - oldLimitImpulse;
			//b2Vec2 P1 = -impulse * m_u1;
			P1X = -impulse * m_u1.x;
			P1Y = -impulse * m_u1.y;
			//b1.m_linearVelocity += b1->m_invMass * P1;
			b1.m_linearVelocity.x += b1.m_invMass * P1X;
			b1.m_linearVelocity.y += b1.m_invMass * P1Y;
			//b1.m_angularVelocity += b1->m_invI * b2Cross(r1, P1);
			b1.m_angularVelocity += b1.m_invI * (r1X * P1Y - r1Y * P1X);
		}
		
		if (m_limitState2 == e_atUpperLimit)
		{
			//b2Vec2 v2 = b2->m_linearVelocity + b2Cross(b2->m_angularVelocity, r2);
			v2X = b2.m_linearVelocity.x + (-b2.m_angularVelocity * r2Y);
			v2Y = b2.m_linearVelocity.y + (b2.m_angularVelocity * r2X);
			
			//float32 Cdot = -b2Dot(m_u2, v2);
			Cdot = -(m_u2.x * v2X + m_u2.y * v2Y);
			impulse = -m_limitMass2 * Cdot;
			oldLimitImpulse = m_limitImpulse2;
			m_limitImpulse2 = b2Math.b2Max(0.0, m_limitImpulse2 + impulse);
			impulse = m_limitImpulse2 - oldLimitImpulse;
			//b2Vec2 P2 = -impulse * m_u2;
			P2X = -impulse * m_u2.x;
			P2Y = -impulse * m_u2.y;
			//b2->m_linearVelocity += b2->m_invMass * P2;
			b2.m_linearVelocity.x += b2.m_invMass * P2X;
			b2.m_linearVelocity.y += b2.m_invMass * P2Y;
			//b2->m_angularVelocity += b2->m_invI * b2Cross(r2, P2);
			b2.m_angularVelocity += b2.m_invI * (r2X * P2Y - r2Y * P2X);
		}
	}
	
	
	
	public override function SolvePositionConstraints():Boolean{
		var b1:b2Body = m_body1;
		var b2:b2Body = m_body2;
		
		var tMat:b2Mat22;
		
		//b2Vec2 s1 = m_ground->m_position + m_groundAnchor1;
		var s1X:Number = m_ground.m_position.x + m_groundAnchor1.x;
		var s1Y:Number = m_ground.m_position.y + m_groundAnchor1.y;
		//b2Vec2 s2 = m_ground->m_position + m_groundAnchor2;
		var s2X:Number = m_ground.m_position.x + m_groundAnchor2.x;
		var s2Y:Number = m_ground.m_position.y + m_groundAnchor2.y;
		
		// temp vars
		var r1X:Number;
		var r1Y:Number;
		var r2X:Number;
		var r2Y:Number;
		var p1X:Number;
		var p1Y:Number;
		var p2X:Number;
		var p2Y:Number;
		var length1:Number;
		var length2:Number;
		var C:Number;
		var impulse:Number;
		var oldLimitPositionImpulse:Number;
		
		var linearError:Number = 0.0;
		
		{
			//var r1:b2Vec2 = b2Mul(b1.m_R, m_localAnchor1);
			tMat = b1.m_R;
			r1X = tMat.col1.x * m_localAnchor1.x + tMat.col2.x * m_localAnchor1.y;
			r1Y = tMat.col1.y * m_localAnchor1.x + tMat.col2.y * m_localAnchor1.y;
			//var r2:b2Vec2 = b2Mul(b2.m_R, m_localAnchor2);
			tMat = b2.m_R;
			r2X = tMat.col1.x * m_localAnchor2.x + tMat.col2.x * m_localAnchor2.y;
			r2Y = tMat.col1.y * m_localAnchor2.x + tMat.col2.y * m_localAnchor2.y;
			
			//b2Vec2 p1 = b1->m_position + r1;
			p1X = b1.m_position.x + r1X;
			p1Y = b1.m_position.y + r1Y;
			//b2Vec2 p2 = b2->m_position + r2;
			p2X = b2.m_position.x + r2X;
			p2Y = b2.m_position.y + r2Y;
			
			// Get the pulley axes.
			//m_u1 = p1 - s1;
			m_u1.Set(p1X - s1X, p1Y - s1Y);
			//m_u2 = p2 - s2;
			m_u2.Set(p2X - s2X, p2Y - s2Y);
			
			length1 = m_u1.Length();
			length2 = m_u2.Length();
			
			if (length1 > b2Settings.b2_linearSlop)
			{
				//m_u1 *= 1.0f / length1;
				m_u1.Multiply( 1.0 / length1 );
			}
			else
			{
				m_u1.SetZero();
			}
			
			if (length2 > b2Settings.b2_linearSlop)
			{
				//m_u2 *= 1.0f / length2;
				m_u2.Multiply( 1.0 / length2 );
			}
			else
			{
				m_u2.SetZero();
			}
			
			C = m_constant - length1 - m_ratio * length2;
			linearError = b2Math.b2Max(linearError, Math.abs(C));
			C = b2Math.b2Clamp(C, -b2Settings.b2_maxLinearCorrection, b2Settings.b2_maxLinearCorrection);
			impulse = -m_pulleyMass * C;
			
			p1X = -impulse * m_u1.x;
			p1Y = -impulse * m_u1.y;
			p2X = -m_ratio * impulse * m_u2.x;
			p2Y = -m_ratio * impulse * m_u2.y;
			
			b1.m_position.x += b1.m_invMass * p1X;
			b1.m_position.y += b1.m_invMass * p1Y;
			b1.m_rotation += b1.m_invI * (r1X * p1Y - r1Y * p1X);
			b2.m_position.x += b2.m_invMass * p2X;
			b2.m_position.y += b2.m_invMass * p2Y;
			b2.m_rotation += b2.m_invI * (r2X * p2Y - r2Y * p2X);
			
			b1.m_R.Set(b1.m_rotation);
			b2.m_R.Set(b2.m_rotation);
		}
		
		if (m_limitState1 == e_atUpperLimit)
		{
			//b2Vec2 r1 = b2Mul(b1->m_R, m_localAnchor1);
			tMat = b1.m_R;
			r1X = tMat.col1.x * m_localAnchor1.x + tMat.col2.x * m_localAnchor1.y;
			r1Y = tMat.col1.y * m_localAnchor1.x + tMat.col2.y * m_localAnchor1.y;
			//b2Vec2 p1 = b1->m_position + r1;
			p1X = b1.m_position.x + r1X;
			p1Y = b1.m_position.y + r1Y;
			
			//m_u1 = p1 - s1;
			m_u1.Set(p1X - s1X, p1Y - s1Y);
			
			length1 = m_u1.Length();
			
			if (length1 > b2Settings.b2_linearSlop)
			{
				//m_u1 *= 1.0 / length1;
				m_u1.x *= 1.0 / length1;
				m_u1.y *= 1.0 / length1;
			}
			else
			{
				m_u1.SetZero();
			}
			
			C = m_maxLength1 - length1;
			linearError = b2Math.b2Max(linearError, -C);
			C = b2Math.b2Clamp(C + b2Settings.b2_linearSlop, -b2Settings.b2_maxLinearCorrection, 0.0);
			impulse = -m_limitMass1 * C;
			oldLimitPositionImpulse = m_limitPositionImpulse1;
			m_limitPositionImpulse1 = b2Math.b2Max(0.0, m_limitPositionImpulse1 + impulse);
			impulse = m_limitPositionImpulse1 - oldLimitPositionImpulse;
			
			//P1 = -impulse * m_u1;
			p1X = -impulse * m_u1.x;
			p1Y = -impulse * m_u1.y;
			
			b1.m_position.x += b1.m_invMass * p1X;
			b1.m_position.y += b1.m_invMass * p1Y;
			//b1.m_rotation += b1.m_invI * b2Cross(r1, P1);
			b1.m_rotation += b1.m_invI * (r1X * p1Y - r1Y * p1X);
			b1.m_R.Set(b1.m_rotation);
		}
		
		if (m_limitState2 == e_atUpperLimit)
		{
			//b2Vec2 r2 = b2Mul(b2->m_R, m_localAnchor2);
			tMat = b2.m_R;
			r2X = tMat.col1.x * m_localAnchor2.x + tMat.col2.x * m_localAnchor2.y;
			r2Y = tMat.col1.y * m_localAnchor2.x + tMat.col2.y * m_localAnchor2.y;
			//b2Vec2 p2 = b2->m_position + r2;
			p2X = b2.m_position.x + r2X;
			p2Y = b2.m_position.y + r2Y;
			
			//m_u2 = p2 - s2;
			m_u2.Set(p2X - s2X, p2Y - s2Y);
			
			length2 = m_u2.Length();
			
			if (length2 > b2Settings.b2_linearSlop)
			{
				//m_u2 *= 1.0 / length2;
				m_u2.x *= 1.0 / length2;
				m_u2.y *= 1.0 / length2;
			}
			else
			{
				m_u2.SetZero();
			}
			
			C = m_maxLength2 - length2;
			linearError = b2Math.b2Max(linearError, -C);
			C = b2Math.b2Clamp(C + b2Settings.b2_linearSlop, -b2Settings.b2_maxLinearCorrection, 0.0);
			impulse = -m_limitMass2 * C;
			oldLimitPositionImpulse = m_limitPositionImpulse2;
			m_limitPositionImpulse2 = b2Math.b2Max(0.0, m_limitPositionImpulse2 + impulse);
			impulse = m_limitPositionImpulse2 - oldLimitPositionImpulse;
			
			//P2 = -impulse * m_u2;
			p2X = -impulse * m_u2.x;
			p2Y = -impulse * m_u2.y;
			
			//b2.m_position += b2.m_invMass * P2;
			b2.m_position.x += b2.m_invMass * p2X;
			b2.m_position.y += b2.m_invMass * p2Y;
			//b2.m_rotation += b2.m_invI * b2Cross(r2, P2);
			b2.m_rotation += b2.m_invI * (r2X * p2Y - r2Y * p2X);
			b2.m_R.Set(b2.m_rotation);
		}
		
		return linearError < b2Settings.b2_linearSlop;
	}
	
	

	public var m_ground:b2Body;
	public var m_groundAnchor1:b2Vec2 = new b2Vec2();
	public var m_groundAnchor2:b2Vec2 = new b2Vec2();
	public var m_localAnchor1:b2Vec2 = new b2Vec2();
	public var m_localAnchor2:b2Vec2 = new b2Vec2();

	public var m_u1:b2Vec2 = new b2Vec2();
	public var m_u2:b2Vec2 = new b2Vec2();
	
	public var m_constant:Number;
	public var m_ratio:Number;
	
	public var m_maxLength1:Number;
	public var m_maxLength2:Number;

	// Effective masses
	public var m_pulleyMass:Number;
	public var m_limitMass1:Number;
	public var m_limitMass2:Number;

	// Impulses for accumulation/warm starting.
	public var m_pulleyImpulse:Number;
	public var m_limitImpulse1:Number;
	public var m_limitImpulse2:Number;

	// Position impulses for accumulation.
	public var m_limitPositionImpulse1:Number;
	public var m_limitPositionImpulse2:Number;

	public var m_limitState1:int;
	public var m_limitState2:int;
	
	// static
	static public var b2_minPulleyLength:Number = b2Settings.b2_lengthUnitsPerMeter;
};
	
	
}