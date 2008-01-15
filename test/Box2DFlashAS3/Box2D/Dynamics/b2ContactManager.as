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


import Box2D.Collision.*
import Box2D.Collision.Shapes.*
import Box2D.Dynamics.Contacts.*
import Box2D.Dynamics.*
import Box2D.Common.Math.*
import Box2D.Common.*;


public class b2ContactManager extends b2PairCallback
{
	public function b2ContactManager() {
		m_world = null;
		m_destroyImmediate = false;
	};

	// This is a callback from the broadphase when two AABB proxies begin
	// to overlap. We create a b2Contact to manage the narrow phase.
	public override function PairAdded(proxyUserData1:*, proxyUserData2:*):*{
		var shape1:b2Shape = proxyUserData1 as b2Shape;
		var shape2:b2Shape = proxyUserData2 as b2Shape;
		
		var body1:b2Body = shape1.m_body;
		var body2:b2Body = shape2.m_body;
		
		if (body1.IsStatic() && body2.IsStatic())
		{
			return m_nullContact;
		}
		
		if (shape1.m_body == shape2.m_body)
		{
			return m_nullContact;
		}
		
		if (body2.IsConnected(body1))
		{
			return m_nullContact;
		}
		
		if (m_world.m_filter != null && m_world.m_filter.ShouldCollide(shape1, shape2) == false)
		{
			return m_nullContact;
		}
		
		// Ensure that body2 is dynamic (body1 is static or dynamic).
		if (body2.m_invMass == 0.0)
		{
			var tempShape:b2Shape = shape1;
			shape1 = shape2;
			shape2 = tempShape;
			//b2Math.b2Swap(shape1, shape2);
			var tempBody:b2Body = body1;
			body1 = body2;
			body2 = tempBody;
			//b2Math.b2Swap(body1, body2);
		}
		
		// Call the factory.
		var contact:b2Contact = b2Contact.Create(shape1, shape2, m_world.m_blockAllocator);
		
		if (contact == null)
		{
			return m_nullContact;
		}
		else
		{
			// Insert into the world.
			contact.m_prev = null;
			contact.m_next = m_world.m_contactList;
			if (m_world.m_contactList != null)
			{
				m_world.m_contactList.m_prev = contact;
			}
			m_world.m_contactList = contact;
			m_world.m_contactCount++;
		}
		
		return contact;
	}

	// This is a callback from the broadphase when two AABB proxies cease
	// to overlap. We destroy the b2Contact.
	public override function PairRemoved(proxyUserData1:*, proxyUserData2:*, pairUserData:*): void{
		
		if (pairUserData == null)
		{
			return;
		}
		
		var c:b2Contact = pairUserData as b2Contact;
		if (c != m_nullContact)
		{
			//b2Settings.b2Assert(m_world.m_contactCount > 0);
			if (m_destroyImmediate == true)
			{
				DestroyContact(c);
				c = null;
			}
			else
			{
				c.m_flags |= b2Contact.e_destroyFlag;
			}
		}
	}

	public function DestroyContact(c:b2Contact) : void
	{
		
		//b2Settings.b2Assert(m_world.m_contactCount > 0);
				
		// Remove from the world.
		if (c.m_prev)
		{
			c.m_prev.m_next = c.m_next;
		}
		
		if (c.m_next)
		{
			c.m_next.m_prev = c.m_prev;
		}
		
		if (c == m_world.m_contactList)
		{
			m_world.m_contactList = c.m_next;
		}
		
		// If there are contact points, then disconnect from the island graph.
		if (c.GetManifoldCount() > 0)
		{
			var body1:b2Body = c.m_shape1.m_body;
			var body2:b2Body = c.m_shape2.m_body;
			var node1:b2ContactNode = c.m_node1;
			var node2:b2ContactNode = c.m_node2;
			
			// Wake up touching bodies.
			body1.WakeUp();
			body2.WakeUp();
			
			// Remove from body 1
			if (node1.prev)
			{
				node1.prev.next = node1.next;
			}
			
			if (node1.next)
			{
				node1.next.prev = node1.prev;
			}
			
			if (node1 == body1.m_contactList)
			{
				body1.m_contactList = node1.next;
			}
			
			node1.prev = null;
			node1.next = null;
			
			// Remove from body 2
			if (node2.prev)
			{
				node2.prev.next = node2.next;
			}
			
			if (node2.next)
			{
				node2.next.prev = node2.prev;
			}
			
			if (node2 == body2.m_contactList)
			{
				body2.m_contactList = node2.next;
			}
			
			node2.prev = null;
			node2.next = null;
		}
		
		// Call the factory.
		b2Contact.Destroy(c, m_world.m_blockAllocator);
		--m_world.m_contactCount;
	}
	
	
	// Destroy any contacts marked for deferred destruction.
	public function CleanContactList() : void
	{
		var c:b2Contact = m_world.m_contactList;
		while (c != null)
		{
			var c0:b2Contact = c;
			c = c.m_next;
			
			if (c0.m_flags & b2Contact.e_destroyFlag)
			{
				DestroyContact(c0);
				c0 = null;
			}
		}
	}
	

	// This is the top level collision call for the time step. Here
	// all the narrow phase collision is processed for the world
	// contact list.
	public function Collide() : void
	{
		var body1:b2Body;
		var body2:b2Body;
		var node1:b2ContactNode;
		var node2:b2ContactNode;
		
		for (var c:b2Contact = m_world.m_contactList; c != null; c = c.m_next)
		{
			if (c.m_shape1.m_body.IsSleeping() &&
				c.m_shape2.m_body.IsSleeping())
			{
				continue;
			}
			
			var oldCount:int = c.GetManifoldCount();
			c.Evaluate();
			
			var newCount:int = c.GetManifoldCount();
			
			if (oldCount == 0 && newCount > 0)
			{
				//b2Settings.b2Assert(c.GetManifolds().pointCount > 0);
				
				// Connect to island graph.
				body1 = c.m_shape1.m_body;
				body2 = c.m_shape2.m_body;
				node1 = c.m_node1;
				node2 = c.m_node2;
				
				// Connect to body 1
				node1.contact = c;
				node1.other = body2;
				
				node1.prev = null;
				node1.next = body1.m_contactList;
				if (node1.next != null)
				{
					node1.next.prev = c.m_node1;
				}
				body1.m_contactList = c.m_node1;
				
				// Connect to body 2
				node2.contact = c;
				node2.other = body1;
				
				node2.prev = null;
				node2.next = body2.m_contactList;
				if (node2.next != null)
				{
					node2.next.prev = node2;
				}
				body2.m_contactList = node2;
			}
			else if (oldCount > 0 && newCount == 0)
			{
				// Disconnect from island graph.
				body1 = c.m_shape1.m_body;
				body2 = c.m_shape2.m_body;
				node1 = c.m_node1;
				node2 = c.m_node2;
				
				// Remove from body 1
				if (node1.prev)
				{
					node1.prev.next = node1.next;
				}
				
				if (node1.next)
				{
					node1.next.prev = node1.prev;
				}
				
				if (node1 == body1.m_contactList)
				{
					body1.m_contactList = node1.next;
				}
				
				node1.prev = null;
				node1.next = null;
				
				// Remove from body 2
				if (node2.prev)
				{
					node2.prev.next = node2.next;
				}
				
				if (node2.next)
				{
					node2.next.prev = node2.prev;
				}
				
				if (node2 == body2.m_contactList)
				{
					body2.m_contactList = node2.next;
				}
				
				node2.prev = null;
				node2.next = null;
			}
		}
	}

	public var m_world:b2World;

	// This lets us provide broadphase proxy pair user data for
	// contacts that shouldn't exist.
	public var m_nullContact:b2NullContact = new b2NullContact();
	public var m_destroyImmediate:Boolean;
	
};

}
