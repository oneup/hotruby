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
import Box2D.Dynamics.Contacts.*
import Box2D.Collision.Shapes.*
import Box2D.Collision.*
import Box2D.Common.Math.*
import Box2D.Common.*


//typedef b2Contact* b2ContactCreateFcn(b2Shape* shape1, b2Shape* shape2, b2BlockAllocator* allocator);
//typedef void b2ContactDestroyFcn(b2Contact* contact, b2BlockAllocator* allocator);



public class b2Contact
{
	public virtual function GetManifolds():Array{return null};
	public function GetManifoldCount():int
	{
		return m_manifoldCount;
	}

	public function GetNext():b2Contact{
		return m_next;
	}

	public function GetShape1():b2Shape{
		return m_shape1;
	}

	public function GetShape2():b2Shape{
		return m_shape2;
	}

	//--------------- Internals Below -------------------
	
	// m_flags
	// enum
	static public var e_islandFlag:uint		= 0x0001;
	static public var e_destroyFlag:uint	= 0x0002;

	static public function AddType(createFcn:*, destroyFcn:*, type1:int, type2:int) : void
	{
		//b2Settings.b2Assert(b2Shape.e_unknownShape < type1 && type1 < b2Shape.e_shapeTypeCount);
		//b2Settings.b2Assert(b2Shape.e_unknownShape < type2 && type2 < b2Shape.e_shapeTypeCount);
		
		s_registers[type1][type2].createFcn = createFcn;
		s_registers[type1][type2].destroyFcn = destroyFcn;
		s_registers[type1][type2].primary = true;
		
		if (type1 != type2)
		{
			s_registers[type2][type1].createFcn = createFcn;
			s_registers[type2][type1].destroyFcn = destroyFcn;
			s_registers[type2][type1].primary = false;
		}
	}
	static public function InitializeRegisters() : void{
		s_registers = new Array(b2Shape.e_shapeTypeCount);
		for (var i:int = 0; i < b2Shape.e_shapeTypeCount; i++){
			s_registers[i] = new Array(b2Shape.e_shapeTypeCount);
			for (var j:int = 0; j < b2Shape.e_shapeTypeCount; j++){
				s_registers[i][j] = new b2ContactRegister();
			}
		}
		
		AddType(b2CircleContact.Create, b2CircleContact.Destroy, b2Shape.e_circleShape, b2Shape.e_circleShape);
		AddType(b2PolyAndCircleContact.Create, b2PolyAndCircleContact.Destroy, b2Shape.e_polyShape, b2Shape.e_circleShape);
		AddType(b2PolyContact.Create, b2PolyContact.Destroy, b2Shape.e_polyShape, b2Shape.e_polyShape);
		
	}
	static public function Create(shape1:b2Shape, shape2:b2Shape, allocator:*):b2Contact{
		if (s_initialized == false)
		{
			InitializeRegisters();
			s_initialized = true;
		}
		
		var type1:int = shape1.m_type;
		var type2:int = shape2.m_type;
		
		//b2Settings.b2Assert(b2Shape.e_unknownShape < type1 && type1 < b2Shape.e_shapeTypeCount);
		//b2Settings.b2Assert(b2Shape.e_unknownShape < type2 && type2 < b2Shape.e_shapeTypeCount);
		
		var createFcn:* = s_registers[type1][type2].createFcn;
		if (createFcn)
		{
			if (s_registers[type1][type2].primary)
			{
				return createFcn(shape1, shape2, allocator);
			}
			else
			{
				var c:b2Contact = createFcn(shape2, shape1, allocator);
				for (var i:int = 0; i < c.GetManifoldCount(); ++i)
				{
					var m:b2Manifold = c.GetManifolds()[ i ];
					m.normal = m.normal.Negative();
				}
				return c;
			}
		}
		else
		{
			return null;
		}
	}
	static public function Destroy(contact:b2Contact, allocator:*) : void{
		//b2Settings.b2Assert(s_initialized == true);
		
		if (contact.GetManifoldCount() > 0)
		{
			contact.m_shape1.m_body.WakeUp();
			contact.m_shape2.m_body.WakeUp();
		}
		
		var type1:int = contact.m_shape1.m_type;
		var type2:int = contact.m_shape2.m_type;
		
		//b2Settings.b2Assert(b2Shape.e_unknownShape < type1 && type1 < b2Shape.e_shapeTypeCount);
		//b2Settings.b2Assert(b2Shape.e_unknownShape < type2 && type2 < b2Shape.e_shapeTypeCount);
		
		var destroyFcn:* = s_registers[type1][type2].destroyFcn;
		destroyFcn(contact, allocator);
	}

	public function b2Contact(s1:b2Shape=null, s2:b2Shape=null)
	{
		m_flags = 0;
		
		if (!s1 || !s2){
			m_shape1 = null;
			m_shape2 = null;
			return;
		}
		
		m_shape1 = s1;
		m_shape2 = s2;
		
		m_manifoldCount = 0;
		
		m_friction = Math.sqrt(m_shape1.m_friction * m_shape2.m_friction);
		m_restitution = b2Math.b2Max(m_shape1.m_restitution, m_shape2.m_restitution);
		
		m_prev = null;
		m_next = null;
		
		m_node1.contact = null;
		m_node1.prev = null;
		m_node1.next = null;
		m_node1.other = null;
		
		m_node2.contact = null;
		m_node2.prev = null;
		m_node2.next = null;
		m_node2.other = null;
	}

	//virtual ~b2Contact() {}

	public virtual function Evaluate() : void{};
	static public var s_registers:Array; //[][]
	static public var s_initialized:Boolean = false;

	public var m_flags:uint;

	// World pool and list pointers.
	public var m_prev:b2Contact;
	public var m_next:b2Contact;

	// Nodes for connecting bodies.
	public var m_node1:b2ContactNode = new b2ContactNode();
	public var m_node2:b2ContactNode = new b2ContactNode();

	public var m_shape1:b2Shape;
	public var m_shape2:b2Shape;

	public var m_manifoldCount:int;

	// Combined friction
	public var m_friction:Number;
	public var m_restitution:Number;

};


}
