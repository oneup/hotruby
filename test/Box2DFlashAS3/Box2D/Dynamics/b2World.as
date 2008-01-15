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

import Box2D.Common.Math.*
import Box2D.Common.*
import Box2D.Collision.*
import Box2D.Collision.Shapes.*
import Box2D.Dynamics.*
import Box2D.Dynamics.Contacts.*
import Box2D.Dynamics.Joints.*


public class b2World
{
	public function b2World(worldAABB:b2AABB, gravity:b2Vec2, doSleep:Boolean){
		
		m_listener = null;
		m_filter = b2CollisionFilter.b2_defaultFilter;
		
		m_bodyList = null;
		m_contactList = null;
		m_jointList = null;
		
		m_bodyCount = 0;
		m_contactCount = 0;
		m_jointCount = 0;
		
		m_bodyDestroyList = null;
		
		m_allowSleep = doSleep;
		
		m_gravity = gravity;
		
		m_contactManager.m_world = this;
		m_broadPhase = new b2BroadPhase(worldAABB, m_contactManager);
		
		var bd:b2BodyDef = new b2BodyDef();
		m_groundBody = CreateBody(bd);
	}
	//~b2World(){
	//	DestroyBody(m_groundBody);
	//	delete m_broadPhase;
	//}

	// Set a callback to notify you when a joint is implicitly destroyed
	// when an attached body is destroyed.
	public function SetListener(listener:b2WorldListener) : void{
		m_listener = listener;
	}
	
	// Register a collision filter to provide specific control over collision.
	// Otherwise the default filter is used (b2CollisionFilter).
	public function SetFilter(filter:b2CollisionFilter) : void{
		m_filter = filter;
	}

	// Create and destroy rigid bodies. Destruction is deferred until the
	// the next call to Step. This is done so that bodies may be destroyed
	// while you iterate through the contact list.
	public function CreateBody(def:b2BodyDef):b2Body{
		//void* mem = m_blockAllocator.Allocate(sizeof(b2Body));
		var b:b2Body = new b2Body(def, this);
		b.m_prev = null;
		
		b.m_next = m_bodyList;
		if (m_bodyList)
		{
			m_bodyList.m_prev = b;
		}
		m_bodyList = b;
		++m_bodyCount;
		
		return b;
	}
	// Body destruction is deferred to make contact processing more robust.
	public function DestroyBody(b:b2Body) : void
	{
		
		if (b.m_flags & b2Body.e_destroyFlag)
		{
			return;
		}
		
		// Remove from normal body list.
		if (b.m_prev)
		{
			b.m_prev.m_next = b.m_next;
		}
		
		if (b.m_next)
		{
			b.m_next.m_prev = b.m_prev;
		}
		
		if (b == m_bodyList)
		{
			m_bodyList = b.m_next;
		}
		
		b.m_flags |= b2Body.e_destroyFlag;
		//b2Settings.b2Assert(m_bodyCount > 0);
		--m_bodyCount;
		
		//b->~b2Body();
		//b.Destroy();
		// Add to the deferred destruction list.
		b.m_prev = null;
		b.m_next = m_bodyDestroyList;
		m_bodyDestroyList = b;
	}
	
	public function CleanBodyList() : void
	{
		m_contactManager.m_destroyImmediate = true;
		
		var b:b2Body = m_bodyDestroyList;
		while (b)
		{
			//b2Settings.b2Assert((b.m_flags & b2Body.e_destroyFlag) != 0);
			
			// Preserve the next pointer.
			var b0:b2Body = b;
			b = b.m_next;
			
			// Delete the attached joints
			var jn:b2JointNode = b0.m_jointList;
			while (jn)
			{
				var jn0:b2JointNode = jn;
				jn = jn.next;
				
				if (m_listener)
				{
					m_listener.NotifyJointDestroyed(jn0.joint);
				}
				
				DestroyJoint(jn0.joint);
			}
			
			b0.Destroy();
			//m_blockAllocator.Free(b0, sizeof(b2Body));
		}
		
		// Reset the list.
		m_bodyDestroyList = null;
		
		m_contactManager.m_destroyImmediate = false;
	}

	public function CreateJoint(def:b2JointDef):b2Joint{
		var j:b2Joint = b2Joint.Create(def, m_blockAllocator);
		
		// Connect to the world list.
		j.m_prev = null;
		j.m_next = m_jointList;
		if (m_jointList)
		{
			m_jointList.m_prev = j;
		}
		m_jointList = j;
		++m_jointCount;
		
		// Connect to the bodies
		j.m_node1.joint = j;
		j.m_node1.other = j.m_body2;
		j.m_node1.prev = null;
		j.m_node1.next = j.m_body1.m_jointList;
		if (j.m_body1.m_jointList) j.m_body1.m_jointList.prev = j.m_node1;
		j.m_body1.m_jointList = j.m_node1;
		
		j.m_node2.joint = j;
		j.m_node2.other = j.m_body1;
		j.m_node2.prev = null;
		j.m_node2.next = j.m_body2.m_jointList;
		if (j.m_body2.m_jointList) j.m_body2.m_jointList.prev = j.m_node2;
		j.m_body2.m_jointList = j.m_node2;
		
		// If the joint prevents collisions, then reset collision filtering.
		if (def.collideConnected == false)
		{
			// Reset the proxies on the body with the minimum number of shapes.
			var b:b2Body = def.body1.m_shapeCount < def.body2.m_shapeCount ? def.body1 : def.body2;
			for (var s:b2Shape = b.m_shapeList; s; s = s.m_next)
			{
				s.ResetProxy(m_broadPhase);
			}
		}
		
		return j;
	}
	public function DestroyJoint(j:b2Joint) : void
	{
		
		var collideConnected:Boolean = j.m_collideConnected;
		
		// Remove from the world.
		if (j.m_prev)
		{
			j.m_prev.m_next = j.m_next;
		}
		
		if (j.m_next)
		{
			j.m_next.m_prev = j.m_prev;
		}
		
		if (j == m_jointList)
		{
			m_jointList = j.m_next;
		}
		
		// Disconnect from island graph.
		var body1:b2Body = j.m_body1;
		var body2:b2Body = j.m_body2;
		
		// Wake up touching bodies.
		body1.WakeUp();
		body2.WakeUp();
		
		// Remove from body 1
		if (j.m_node1.prev)
		{
			j.m_node1.prev.next = j.m_node1.next;
		}
		
		if (j.m_node1.next)
		{
			j.m_node1.next.prev = j.m_node1.prev;
		}
		
		if (j.m_node1 == body1.m_jointList)
		{
			body1.m_jointList = j.m_node1.next;
		}
		
		j.m_node1.prev = null;
		j.m_node1.next = null;
		
		// Remove from body 2
		if (j.m_node2.prev)
		{
			j.m_node2.prev.next = j.m_node2.next;
		}
		
		if (j.m_node2.next)
		{
			j.m_node2.next.prev = j.m_node2.prev;
		}
		
		if (j.m_node2 == body2.m_jointList)
		{
			body2.m_jointList = j.m_node2.next;
		}
		
		j.m_node2.prev = null;
		j.m_node2.next = null;
		
		b2Joint.Destroy(j, m_blockAllocator);
		
		//b2Settings.b2Assert(m_jointCount > 0);
		--m_jointCount;
		
		// If the joint prevents collisions, then reset collision filtering.
		if (collideConnected == false)
		{
			// Reset the proxies on the body with the minimum number of shapes.
			var b:b2Body = body1.m_shapeCount < body2.m_shapeCount ? body1 : body2;
			for (var s:b2Shape = b.m_shapeList; s; s = s.m_next)
			{
				s.ResetProxy(m_broadPhase);
			}
		}
	}

	// The world provides a single ground body with no collision shapes. You
	// can use this to simplify the creation of joints.
	public function GetGroundBody():b2Body{
		return m_groundBody;
	}


	private var step:b2TimeStep = new b2TimeStep();
	// Step
	public function Step(dt:Number, iterations:int) : void{
		
		var b:b2Body;
		var other:b2Body;
		
		
		step.dt = dt;
		step.iterations	= iterations;
		if (dt > 0.0)
		{
			step.inv_dt = 1.0 / dt;
		}
		else
		{
			step.inv_dt = 0.0;
		}
		
		m_positionIterationCount = 0;
		
		// Handle deferred contact destruction.
		m_contactManager.CleanContactList();
		
		// Handle deferred body destruction.
		CleanBodyList();
		
		// Update contacts.
		m_contactManager.Collide();
		
		// Size the island for the worst case.
		var island:b2Island = new b2Island(m_bodyCount, m_contactCount, m_jointCount, m_stackAllocator);
		
		// Clear all the island flags.
		for (b = m_bodyList; b != null; b = b.m_next)
		{
			b.m_flags &= ~b2Body.e_islandFlag;
		}
		for (var c:b2Contact = m_contactList; c != null; c = c.m_next)
		{
			c.m_flags &= ~b2Contact.e_islandFlag;
		}
		for (var j:b2Joint = m_jointList; j != null; j = j.m_next)
		{
			j.m_islandFlag = false;
		}
		
		// Build and simulate all awake islands.
		var stackSize:int = m_bodyCount;
		//var stack:b2Body = (b2Body**)m_stackAllocator.Allocate(stackSize * sizeof(b2Body*));
		var stack:Array = new Array(m_bodyCount);
		for (var k:int = 0; k < m_bodyCount; k++)
			stack[k] = null;
		
		for (var seed:b2Body = m_bodyList; seed != null; seed = seed.m_next)
		{
			if (seed.m_flags & (b2Body.e_staticFlag | b2Body.e_islandFlag | b2Body.e_sleepFlag | b2Body.e_frozenFlag))
			{
				continue;
			}
			
			// Reset island and stack.
			island.Clear();
			var stackCount:int = 0;
			stack[stackCount++] = seed;
			seed.m_flags |= b2Body.e_islandFlag;;
			
			// Perform a depth first search (DFS) on the constraint graph.
			while (stackCount > 0)
			{
				// Grab the next body off the stack and add it to the island.
				b = stack[--stackCount];
				island.AddBody(b);
				
				// Make sure the body is awake.
				b.m_flags &= ~b2Body.e_sleepFlag;
				
				// To keep islands as small as possible, we don't
				// propagate islands across static bodies.
				if (b.m_flags & b2Body.e_staticFlag)
				{
					continue;
				}
				
				// Search all contacts connected to this body.
				for (var cn:b2ContactNode = b.m_contactList; cn != null; cn = cn.next)
				{
					if (cn.contact.m_flags & b2Contact.e_islandFlag)
					{
						continue;
					}
					
					island.AddContact(cn.contact);
					cn.contact.m_flags |= b2Contact.e_islandFlag;
					
					other = cn.other;
					if (other.m_flags & b2Body.e_islandFlag)
					{
						continue;
					}
					
					//b2Settings.b2Assert(stackCount < stackSize);
					stack[stackCount++] = other;
					other.m_flags |= b2Body.e_islandFlag;
				}
				
				// Search all joints connect to this body.
				for (var jn:b2JointNode = b.m_jointList; jn != null; jn = jn.next)
				{
					if (jn.joint.m_islandFlag == true)
					{
						continue;
					}
					
					island.AddJoint(jn.joint);
					jn.joint.m_islandFlag = true;
					
					other = jn.other;
					if (other.m_flags & b2Body.e_islandFlag)
					{
						continue;
					}
					
					//b2Settings.b2Assert(stackCount < stackSize);
					stack[stackCount++] = other;
					other.m_flags |= b2Body.e_islandFlag;
				}
			}
			
			island.Solve(step, m_gravity);
			
			m_positionIterationCount = b2Math.b2Max(m_positionIterationCount, b2Island.m_positionIterationCount);
			
			if (m_allowSleep)
			{
				island.UpdateSleep(dt);
			}
			
			// Post solve cleanup.
			for (var i:int = 0; i < island.m_bodyCount; ++i)
			{
				// Allow static bodies to participate in other islands.
				b = island.m_bodies[i];
				if (b.m_flags & b2Body.e_staticFlag)
				{
					b.m_flags &= ~b2Body.e_islandFlag;
				}
				
				// Handle newly frozen bodies.
				if (b.IsFrozen() && m_listener)
				{
					var response:uint = m_listener.NotifyBoundaryViolated(b);
					if (response == b2WorldListener.b2_destroyBody)
					{
						DestroyBody(b);
						b = null;
						island.m_bodies[i] = null;
					}
				}
			}
		}
		
		m_broadPhase.Commit();
		
		//m_stackAllocator.Free(stack);
	}

	// Query the world for all shapes that potentially overlap the
	// provided AABB. You provide a shape pointer buffer of specified
	// size. The number of shapes found is returned.
	public function Query(aabb:b2AABB, shapes:Array, maxCount:int):int{
		
		//void** results = (void**)m_stackAllocator.Allocate(maxCount * sizeof(void*));
		var results:Array = new Array();
		var count:int = m_broadPhase.QueryAABB(aabb, results, maxCount);
		
		for (var i:int = 0; i < count; ++i)
		{
			shapes[i] = results[i] as b2Shape;
		}
		
		//m_stackAllocator.Free(results);
		return count;
	}

	// You can use these to iterate over all the bodies, joints, and contacts.
	public function GetBodyList():b2Body{
		return m_bodyList;
	}
	public function GetJointList():b2Joint{
		return m_jointList;
	}
	public function GetContactList():b2Contact{
		return m_contactList;
	}

	//--------------- Internals Below -------------------

	public var m_blockAllocator:*;
	public var m_stackAllocator:*;

	public var m_broadPhase:b2BroadPhase;
	public var m_contactManager:b2ContactManager = new b2ContactManager();

	public var m_bodyList:b2Body;
	public var m_contactList:b2Contact;
	public var m_jointList:b2Joint;

	public var m_bodyCount:int;
	public var m_contactCount:int;
	public var m_jointCount:int;
	
	// These bodies will be destroyed at the next time step.
	public var m_bodyDestroyList:b2Body;

	public var m_gravity:b2Vec2;
	public var m_allowSleep:Boolean;

	public var m_groundBody:b2Body;
	
	public var m_listener:b2WorldListener;
	public var m_filter:b2CollisionFilter;
	
	public var m_positionIterationCount:int;
	
	static public var s_enablePositionCorrection:int = 1;
	static public var s_enableWarmStarting:int = 1;
};



}
