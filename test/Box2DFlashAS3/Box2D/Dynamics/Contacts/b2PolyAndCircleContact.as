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


import Box2D.Collision.Shapes.*
import Box2D.Collision.*
import Box2D.Dynamics.*
import Box2D.Common.*
import Box2D.Common.Math.*


public class b2PolyAndCircleContact extends b2Contact{
	
	static public function Create(shape1:b2Shape, shape2:b2Shape, allocator:*):b2Contact{
		return new b2PolyAndCircleContact(shape1, shape2);
	}
	static public function Destroy(contact:b2Contact, allocator:*): void{
		//
	}

	public function b2PolyAndCircleContact(shape1:b2Shape, shape2:b2Shape){
		super(shape1, shape2);
		
		b2Settings.b2Assert(m_shape1.m_type == b2Shape.e_polyShape);
		b2Settings.b2Assert(m_shape2.m_type == b2Shape.e_circleShape);
		m_manifold[0].pointCount = 0;
		m_manifold[0].points[0].normalImpulse = 0.0;
		m_manifold[0].points[0].tangentImpulse = 0.0;
	}
	//~b2PolyAndCircleContact() {}

	public override function Evaluate(): void{
		b2Collision.b2CollidePolyAndCircle(m_manifold[0], m_shape1 as b2PolyShape, m_shape2 as b2CircleShape, false);
		
		if (m_manifold[0].pointCount > 0)
		{
			m_manifoldCount = 1;
		}
		else
		{
			m_manifoldCount = 0;
		}
	}
	
	public override function GetManifolds():Array
	{
		return m_manifold;
	}

	public var m_manifold:Array = [new b2Manifold()];
	
}

}