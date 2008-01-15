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

package Box2D.Dynamics{


import Box2D.Dynamics.*
import Box2D.Dynamics.Joints.*
import Box2D.Dynamics.Contacts.*
import Box2D.Collision.Shapes.*
import Box2D.Common.Math.*
import Box2D.Common.*



// A rigid body. Internal computation are done in terms
// of the center of mass position. The center of mass may
// be offset from the body's origin.
public class b2Body
{
	// Set the position of the body's origin and rotation (radians).
	// This breaks any contacts and wakes the other bodies.
	public function SetOriginPosition(position:b2Vec2, rotation:Number) : void{
		if (IsFrozen())
		{
			return;
		}
		
		m_rotation = rotation;
		m_R.Set(m_rotation);
		m_position = b2Math.AddVV(position , b2Math.b2MulMV(m_R, m_center));
		
		m_position0.SetV(m_position);
		m_rotation0 = m_rotation;
		
		for (var s:b2Shape = m_shapeList; s != null; s = s.m_next)
		{
			s.Synchronize(m_position, m_R, m_position, m_R);
		}
		
		m_world.m_broadPhase.Commit();
	}

	// Get the position of the body's origin. The body's origin does not
	// necessarily coincide with the center of mass. It depends on how the
	// shapes are created.
	public function GetOriginPosition():b2Vec2{
		return b2Math.SubtractVV(m_position, b2Math.b2MulMV(m_R, m_center));
	}

	// Set the position of the body's center of mass and rotation (radians).
	// This breaks any contacts and wakes the other bodies.
	public function SetCenterPosition(position:b2Vec2, rotation:Number) : void{
		if (IsFrozen())
		{
			return;
		}
		
		m_rotation = rotation;
		m_R.Set(m_rotation);
		m_position.SetV( position );
		
		m_position0.SetV(m_position);
		m_rotation0 = m_rotation;
		
		for (var s:b2Shape = m_shapeList; s != null; s = s.m_next)
		{
			s.Synchronize(m_position, m_R, m_position, m_R);
		}
		
		m_world.m_broadPhase.Commit();
	}

	// Get the position of the body's center of mass. The body's center of mass
	// does not necessarily coincide with the body's origin. It depends on how the
	// shapes are created.
	public function GetCenterPosition():b2Vec2{
		return m_position;
	}

	// Get the rotation in radians.
	public function GetRotation():Number{
		return m_rotation;
	}

	public function GetRotationMatrix():b2Mat22{
		return m_R;
	}

	// Set/Get the linear velocity of the center of mass.
	public function SetLinearVelocity(v:b2Vec2) : void{
		m_linearVelocity.SetV(v);
	}
	public function GetLinearVelocity():b2Vec2{
		return m_linearVelocity;
	}

	// Set/Get the angular velocity.
	public function SetAngularVelocity(w:Number) : void{
		m_angularVelocity = w;
	}
	public function GetAngularVelocity():Number{
		return m_angularVelocity;
	}

	// Apply a force at a world point. Additive.
	public function ApplyForce(force:b2Vec2, point:b2Vec2) : void
	{
		if (IsSleeping() == false)
		{
			m_force.Add( force );
			m_torque += b2Math.b2CrossVV(b2Math.SubtractVV(point, m_position), force);
		}
	}

	// Apply a torque. Additive.
	public function ApplyTorque(torque:Number) : void
	{
		if (IsSleeping() == false)
		{
			m_torque += torque;
		}
	}

	// Apply an impulse at a point. This immediately modifies the velocity.
	public function ApplyImpulse(impulse:b2Vec2, point:b2Vec2) : void
	{
		if (IsSleeping() == false)
		{
			m_linearVelocity.Add( b2Math.MulFV(m_invMass, impulse) );
			m_angularVelocity += ( m_invI * b2Math.b2CrossVV( b2Math.SubtractVV(point, m_position), impulse)  );
		}
	}

	public function GetMass():Number{
		return m_mass;
	}

	public function GetInertia():Number{
		return m_I;
	}

	// Get the world coordinates of a point give the local coordinates
	// relative to the body's center of mass.
	public function GetWorldPoint(localPoint:b2Vec2):b2Vec2{
		return b2Math.AddVV(m_position , b2Math.b2MulMV(m_R, localPoint));
	}

	// Get the world coordinates of a vector given the local coordinates.
	public function GetWorldVector(localVector:b2Vec2):b2Vec2{
		return b2Math.b2MulMV(m_R, localVector);
	}

	// Returns a local point relative to the center of mass given a world point.
	public function GetLocalPoint(worldPoint:b2Vec2):b2Vec2{
		return b2Math.b2MulTMV(m_R, b2Math.SubtractVV(worldPoint, m_position));
	}

	// Returns a local vector given a world vector.
	public function GetLocalVector(worldVector:b2Vec2):b2Vec2{
		return b2Math.b2MulTMV(m_R, worldVector);
	}

	// Is this body static (immovable)?
	public function IsStatic():Boolean{
		return (m_flags & e_staticFlag) == e_staticFlag;
	}
	
	public function IsFrozen():Boolean
	{
		return (m_flags & e_frozenFlag) == e_frozenFlag;
	}

	// Is this body sleeping (not simulating).
	public function IsSleeping():Boolean{
		return (m_flags & e_sleepFlag) == e_sleepFlag;
	}

	// You can disable sleeping on this particular body.
	public function AllowSleeping(flag:Boolean) : void
	{
		if (flag)
		{
			m_flags |= e_allowSleepFlag;
		}
		else
		{
			m_flags &= ~e_allowSleepFlag;
			WakeUp();
		}
	}

	// Wake up this body so it will begin simulating.
	public function WakeUp() : void{
		m_flags &= ~e_sleepFlag;
		m_sleepTime = 0.0;
	}

	// Get the list of all shapes attached to this body.
	public function GetShapeList():b2Shape{
		return m_shapeList;
	}
	
	public function GetContactList():b2ContactNode
	{
		return m_contactList;
	}

	public function GetJointList():b2JointNode
	{
		return m_jointList;
	}

	// Get the next body in the world's body list.
	public function GetNext():b2Body{
		return m_next;
	}

	public function GetUserData():*{
		return m_userData;
	}

	//--------------- Internals Below -------------------

	public function b2Body(bd:b2BodyDef, world:b2World){
		var i:int;
		var sd:b2ShapeDef;
		var massData:b2MassData;
		
		m_flags = 0;
		m_position.SetV( bd.position );
		m_rotation = bd.rotation;
		m_R.Set(m_rotation);
		m_position0.SetV(m_position);
		m_rotation0 = m_rotation;
		m_world = world;
		
		m_linearDamping = b2Math.b2Clamp(1.0 - bd.linearDamping, 0.0, 1.0);
		m_angularDamping = b2Math.b2Clamp(1.0 - bd.angularDamping, 0.0, 1.0);
		
		m_force = new b2Vec2(0.0, 0.0);
		m_torque = 0.0;
		
		m_mass = 0.0;
		
		var massDatas:Array = new Array(b2Settings.b2_maxShapesPerBody);
		for (i = 0; i < b2Settings.b2_maxShapesPerBody; i++){
			massDatas[i] = new b2MassData();
		}
		
		// Compute the shape mass properties, the bodies total mass and COM.
		m_shapeCount = 0;
		m_center = new b2Vec2(0.0, 0.0);
		for (i = 0; i < b2Settings.b2_maxShapesPerBody; ++i)
		{
			sd = bd.shapes[i];
			if (sd == null) break;
			massData = massDatas[ i ];
			sd.ComputeMass(massData);
			m_mass += massData.mass;
			//m_center += massData->mass * (sd->localPosition + massData->center);
			m_center.x += massData.mass * (sd.localPosition.x + massData.center.x);
			m_center.y += massData.mass * (sd.localPosition.y + massData.center.y);
			++m_shapeCount;
		}
		
		// Compute center of mass, and shift the origin to the COM.
		if (m_mass > 0.0)
		{
			m_center.Multiply( 1.0 / m_mass );
			m_position.Add( b2Math.b2MulMV(m_R, m_center) );
		}
		else
		{
			m_flags |= e_staticFlag;
		}
		
		// Compute the moment of inertia.
		m_I = 0.0;
		for (i = 0; i < m_shapeCount; ++i)
		{
			sd = bd.shapes[i];
			massData = massDatas[ i ];
			m_I += massData.I;
			var r:b2Vec2 = b2Math.SubtractVV( b2Math.AddVV(sd.localPosition, massData.center), m_center );
			m_I += massData.mass * b2Math.b2Dot(r, r);
		}
		
		if (m_mass > 0.0)
		{
			m_invMass = 1.0 / m_mass;
		}
		else
		{
			m_invMass = 0.0;
		}
		
		if (m_I > 0.0 && bd.preventRotation == false)
		{
			m_invI = 1.0 / m_I;
		}
		else
		{
			m_I = 0.0;
			m_invI = 0.0;
		}
		
		// Compute the center of mass velocity.
		m_linearVelocity = b2Math.AddVV(bd.linearVelocity, b2Math.b2CrossFV(bd.angularVelocity, m_center));
		m_angularVelocity = bd.angularVelocity;
		
		m_jointList = null;
		m_contactList = null;
		m_prev = null;
		m_next = null;
		
		// Create the shapes.
		m_shapeList = null;
		for (i = 0; i < m_shapeCount; ++i)
		{
			sd = bd.shapes[i];
			var shape:b2Shape = b2Shape.Create(sd, this, m_center);
			shape.m_next = m_shapeList;
			m_shapeList = shape;
		}
		
		m_sleepTime = 0.0;
		if (bd.allowSleep)
		{
			m_flags |= e_allowSleepFlag;
		}
		if (bd.isSleeping)
		{
			m_flags |= e_sleepFlag;
		}
		
		if ((m_flags & e_sleepFlag)  || m_invMass == 0.0)
		{
			m_linearVelocity.Set(0.0, 0.0);
			m_angularVelocity = 0.0;
		}
		
		m_userData = bd.userData;
	}
	// does not support destructors
	/*~b2Body(){
		b2Shape* s = m_shapeList;
		while (s)
		{
			b2Shape* s0 = s;
			s = s->m_next;
			
			b2Shape::Destroy(s0);
		}
	}*/
	
	public function Destroy() : void{
		var s:b2Shape = m_shapeList;
		while (s)
		{
			var s0:b2Shape = s;
			s = s.m_next;
			
			b2Shape.Destroy(s0);
		}
	}

	// Temp mat
	private var sMat0:b2Mat22 = new b2Mat22()
	public function SynchronizeShapes() : void{
		//b2Mat22 R0(m_rotation0);
		sMat0.Set(m_rotation0);
		for (var s:b2Shape = m_shapeList; s != null; s = s.m_next)
		{
			s.Synchronize(m_position0, sMat0, m_position, m_R);
		}
	}
	
	public function QuickSyncShapes() : void{
		for (var s:b2Shape = m_shapeList; s != null; s = s.m_next)
		{
			s.QuickSync(m_position, m_R);
		}
	}

	// This is used to prevent connected bodies from colliding.
	// It may lie, depending on the collideConnected flag.
	public function IsConnected(other:b2Body):Boolean{
		for (var jn:b2JointNode = m_jointList; jn != null; jn = jn.next)
		{
			if (jn.other == other)
				return jn.joint.m_collideConnected == false;
		}
		
		return false;
	}
	
	public function Freeze() : void{
		m_flags |= e_frozenFlag;
		m_linearVelocity.SetZero();
		m_angularVelocity = 0.0;
		
		for (var s:b2Shape = m_shapeList; s != null; s = s.m_next)
		{
			s.DestroyProxy();
		}
	}
	
	public var m_flags:uint;
	
	public var m_position:b2Vec2 = new b2Vec2();	// center of mass position
	public var m_rotation:Number;
	public var m_R:b2Mat22 = new b2Mat22(0);
	
	// Conservative advancement data.
	public var m_position0:b2Vec2 = new b2Vec2();
	public var m_rotation0:Number;

	public var m_linearVelocity:b2Vec2;
	public var m_angularVelocity:Number;

	public var m_force:b2Vec2;
	public var m_torque:Number;

	public var m_center:b2Vec2;	// local vector from client origin to center of mass

	public var m_world:b2World;
	public var m_prev:b2Body;
	public var m_next:b2Body;

	public var m_shapeList:b2Shape;
	public var m_shapeCount:int;

	public var m_jointList:b2JointNode;
	public var m_contactList:b2ContactNode;

	public var m_mass:Number;
	public var m_invMass:Number;
	public var m_I:Number;
	public var m_invI:Number;
	
	public var  m_linearDamping:Number;
	public var  m_angularDamping:Number;

	public var m_sleepTime:Number;

	public var m_userData:*;
	
	
	
	static public var e_staticFlag:uint		= 0x0001;
	static public var e_frozenFlag:uint		= 0x0002;
	static public var e_islandFlag:uint		= 0x0004;
	static public var e_sleepFlag:uint		= 0x0008;
	static public var e_allowSleepFlag:uint	= 0x0010;
	static public var e_destroyFlag:uint	= 0x0020;
};

}
