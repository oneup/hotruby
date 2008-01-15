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


public class b2PolyContact extends b2Contact
{
	static public function Create(shape1:b2Shape, shape2:b2Shape, allocator:*):b2Contact{
		//void* mem = allocator->Allocate(sizeof(b2PolyContact));
		return new b2PolyContact(shape1, shape2);
	}
	static public function Destroy(contact:b2Contact, allocator:*): void{
		//((b2PolyContact*)contact)->~b2PolyContact();
		//allocator->Free(contact, sizeof(b2PolyContact));
	}

	public function b2PolyContact(shape1:b2Shape, shape2:b2Shape): void{
		super(shape1, shape2);
		//b2Settings.b2Assert(m_shape1.m_type == b2Shape.e_polyShape);
		//b2Settings.b2Assert(m_shape2.m_type == b2Shape.e_polyShape);
		m_manifold[0].pointCount = 0;
	}
	//~b2PolyContact() {}

	// store temp manifold to reduce calls to new
	private var m0:b2Manifold = new b2Manifold();
	
	public override function Evaluate(): void{
		var tMani:b2Manifold = m_manifold[0];
		// replace memcpy
		// memcpy(&m0, &m_manifold, sizeof(b2Manifold));
		//m0.points = new Array(tMani.pointCount);
		var tPoints:Array = m0.points;
		
		for (var k:int = 0; k < tMani.pointCount; k++){
			var tPoint:b2ContactPoint = tPoints[k];
			var tPoint0:b2ContactPoint = tMani.points[k];
			//tPoint.separation = tPoint0.separation;
			tPoint.normalImpulse = tPoint0.normalImpulse;
			tPoint.tangentImpulse = tPoint0.tangentImpulse;
			//tPoint.position.SetV( tPoint0.position );
			
			tPoint.id = tPoint0.id.Copy();
			
			/*m0.points[k].id.features = new Features();
			m0.points[k].id.features.referenceFace = m_manifold[0].points[k].id.features.referenceFace;
			m0.points[k].id.features.incidentEdge = m_manifold[0].points[k].id.features.incidentEdge;
			m0.points[k].id.features.incidentVertex = m_manifold[0].points[k].id.features.incidentVertex;
			m0.points[k].id.features.flip = m_manifold[0].points[k].id.features.flip;*/
		}
		//m0.normal.SetV( tMani.normal );
		m0.pointCount = tMani.pointCount;
		
		b2Collision.b2CollidePoly(tMani, m_shape1 as b2PolyShape, m_shape2 as b2PolyShape, false);
		
		// Match contact ids to facilitate warm starting.
		if (tMani.pointCount > 0)
		{
			var match:Array = [false, false];
			
			// Match old contact ids to new contact ids and copy the
			// stored impulses to warm start the solver.
			for (var i:int = 0; i < tMani.pointCount; ++i)
			{
				var cp:b2ContactPoint = tMani.points[ i ];
				
				cp.normalImpulse = 0.0;
				cp.tangentImpulse = 0.0;
				var idKey:uint = cp.id.key;
				
				for (var j:int = 0; j < m0.pointCount; ++j)
				{
					
					if (match[j] == true)
						continue;
					
					var cp0:b2ContactPoint = m0.points[j];
					var id0:b2ContactID = cp0.id;
					
					if (id0.key == idKey)
					{
						match[j] = true;
						cp.normalImpulse = cp0.normalImpulse;
						cp.tangentImpulse = cp0.tangentImpulse;
						break;
					}
				}
			}
			
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
};

}
