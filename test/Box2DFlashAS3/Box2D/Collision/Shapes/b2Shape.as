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

package Box2D.Collision.Shapes{




import Box2D.Common.Math.*;
import Box2D.Common.*
import Box2D.Dynamics.*
import Box2D.Collision.*



// Shapes are created automatically when a body is created.
// Client code does not normally interact with shapes.
public class b2Shape
{
	public virtual function TestPoint(p:b2Vec2):Boolean{return false};
	
	public function GetUserData():* {return m_userData;};

	public function GetType():int{
		return m_type;
	}

	// Get the parent body of this shape.
	public function GetBody():b2Body{
		return m_body;
	}

	public function GetPosition():b2Vec2{
		return m_position;
	}
	public function GetRotationMatrix():b2Mat22{
		return m_R;
	}
	
	// Remove and then add proxy from the broad-phase.
	// This is used to refresh the collision filters.
	public virtual function ResetProxy(broadPhase:b2BroadPhase) : void{};

	// Get the next shape in the parent body's shape list.
	public function GetNext():b2Shape{
		return m_next;
	}

	//--------------- Internals Below -------------------

	static public function Create(def:b2ShapeDef, body:b2Body, center:b2Vec2):b2Shape{
		switch (def.type)
		{
		case e_circleShape:
			{
				//void* mem = body->m_world->m_blockAllocator.Allocate(sizeof(b2CircleShape));
				return new b2CircleShape(def, body, center);
			}
		
		case e_boxShape:
		case e_polyShape:
			{
				//void* mem = body->m_world->m_blockAllocator.Allocate(sizeof(b2PolyShape));
				return new b2PolyShape(def, body, center);
			}
		}
		
		//b2Settings.b2Assert(false);
		return null;
	}

	static public function Destroy(shape:b2Shape) : void
	{
		/*b2BlockAllocator& allocator = shape->m_body->m_world->m_blockAllocator;
		
		switch (shape.m_type)
		{
		case e_circleShape:
			shape->~b2Shape();
			allocator.Free(shape, sizeof(b2CircleShape));
			break;
		
		case e_polyShape:
			shape->~b2Shape();
			allocator.Free(shape, sizeof(b2PolyShape));
			break;
		
		default:
			b2Assert(false);
		}
		
		shape = NULL;*/
		
		// FROM DESTRUCTOR
		if (shape.m_proxyId != b2Pair.b2_nullProxy)
			shape.m_body.m_world.m_broadPhase.DestroyProxy(shape.m_proxyId);
	}


	public function b2Shape(def:b2ShapeDef, body:b2Body){
		m_userData = def.userData;
		
		m_friction = def.friction;
		m_restitution = def.restitution;
		m_body = body;
		
		m_proxyId = b2Pair.b2_nullProxy;
		
		m_maxRadius = 0.0;
		
		m_categoryBits = def.categoryBits;
		m_maskBits = def.maskBits;
		m_groupIndex = def.groupIndex;
	}

	// Internal use only. Do not call.
	//b2Shape::~b2Shape()
	//{
	//	m_body->m_world->m_broadPhase->DestroyProxy(m_proxyId);
	//}
	
	
	public function DestroyProxy() : void
	{
		if (m_proxyId != b2Pair.b2_nullProxy)
		{
			m_body.m_world.m_broadPhase.DestroyProxy(m_proxyId);
			m_proxyId = b2Pair.b2_nullProxy;
		}
	}
	

	// Internal use only. Do not call.
	public virtual function Synchronize(position1:b2Vec2, R1:b2Mat22,
										position2:b2Vec2, R2:b2Mat22) : void{};
	public virtual function QuickSync(position:b2Vec2, R:b2Mat22) : void{};
	public virtual function Support(dX:Number, dY:Number, out:b2Vec2) : void{};
	public function GetMaxRadius():Number{
		return m_maxRadius;
	}

	public var m_next:b2Shape;
	
	public var m_R:b2Mat22 = new b2Mat22();
	public var m_position:b2Vec2 = new b2Vec2();

	public var m_type:int;

	public var m_userData:* = null;

	public var m_body:b2Body;

	public var m_friction:Number;
	public var m_restitution:Number;

	public var m_maxRadius:Number;

	public var m_proxyId:uint;
	public var m_categoryBits:uint;
	public var m_maskBits:uint;
	public var m_groupIndex:int;
	
	
	
	// b2ShapeType
	static public const e_unknownShape:int = 	-1;
	static public const e_circleShape:int = 	0;
	static public const e_boxShape:int = 		1;
	static public const e_polyShape:int = 		2;
	static public const e_meshShape:int = 		3;
	static public const e_shapeTypeCount:int = 	4;
	
	
	
	
	
	
	
	static public function PolyMass(massData:b2MassData, vs:Array, count:int, rho:Number) : void
	{
		//b2Settings.b2Assert(count >= 3);
		
		//var center:b2Vec2 = new b2Vec2(0.0, 0.0);
		var center:b2Vec2 = new b2Vec2();
		center.SetZero();
		
		var area:Number = 0.0;
		var I:Number = 0.0;
		
		// pRef is the reference point for forming triangles.
		// It's location doesn't change the result (except for rounding error).
		var pRef:b2Vec2 = new b2Vec2(0.0, 0.0);
		
		const inv3:Number = 1.0 / 3.0;
		
		for (var i:int = 0; i < count; ++i)
		{
			// Triangle vertices.
			var p1:b2Vec2 = pRef;
			var p2:b2Vec2 = vs[i];
			var p3:b2Vec2 = i + 1 < count ? vs[i+1] : vs[0];
			
			var e1:b2Vec2 = b2Math.SubtractVV(p2, p1);
			var e2:b2Vec2 = b2Math.SubtractVV(p3, p1);
			
			var D:Number = b2Math.b2CrossVV(e1, e2);
			
			var triangleArea:Number = 0.5 * D;
			area += triangleArea;
			
			// Area weighted centroid
			// center += triangleArea * inv3 * (p1 + p2 + p3);
			var tVec:b2Vec2 = new b2Vec2();
			tVec.SetV(p1);
			tVec.Add(p2);
			tVec.Add(p3);
			tVec.Multiply(inv3*triangleArea);
			center.Add(tVec);
			
			var px:Number = p1.x;
			var py:Number = p1.y;
			var ex1:Number = e1.x; 
			var ey1:Number = e1.y;
			var ex2:Number = e2.x;
			var ey2:Number = e2.y;
			
			var intx2:Number = inv3 * (0.25 * (ex1*ex1 + ex2*ex1 + ex2*ex2) + (px*ex1 + px*ex2)) + 0.5*px*px;
			var inty2:Number = inv3 * (0.25 * (ey1*ey1 + ey2*ey1 + ey2*ey2) + (py*ey1 + py*ey2)) + 0.5*py*py;
			
			I += D * (intx2 + inty2);
		}
		
		// Total mass
		massData.mass = rho * area;
		
		// Center of mass
		//b2Settings.b2Assert(area > Number.MIN_VALUE);
		center.Multiply( 1.0 / area );
		massData.center = center;
		
		// Inertia tensor relative to the center.
		I = rho * (I - area * b2Math.b2Dot(center, center));
		massData.I = I;
	}
	
	
	static public function PolyCentroid(vs:Array, count:int, out:b2Vec2):void
	{
		//b2Settings.b2Assert(count >= 3);
		
		//b2Vec2 c; c.Set(0.0f, 0.0f);
		var cX:Number = 0.0;
		var cY:Number = 0.0;
		//float32 area = 0.0f;
		var area:Number = 0.0;
		
		// pRef is the reference point for forming triangles.
		// It's location doesn't change the result (except for rounding error).
		//b2Vec2 pRef(0.0f, 0.0f);
		var pRefX:Number = 0.0;
		var pRefY:Number = 0.0;
	/*
		// This code would put the reference point inside the polygon.
		for (var i:int = 0; i < count; ++i)
		{
			//pRef += vs[i];
			pRef.x += vs[i].x;
			pRef.y += vs[i].y;
		}
		pRef.x *= 1.0 / count;
		pRef.y *= 1.0 / count;
	*/
		
		//const float32 inv3 = 1.0f / 3.0f;
		const inv3:Number = 1.0 / 3.0;
		
		for (var i:int = 0; i < count; ++i)
		{
			// Triangle vertices.
			//b2Vec2 p1 = pRef;
			var p1X:Number = pRefX;
			var p1Y:Number = pRefY;
			//b2Vec2 p2 = vs[i];
			var p2X:Number = vs[i].x;
			var p2Y:Number = vs[i].y;
			//b2Vec2 p3 = i + 1 < count ? vs[i+1] : vs[0];
			var p3X:Number = i + 1 < count ? vs[i+1].x : vs[0].x;
			var p3Y:Number = i + 1 < count ? vs[i+1].y : vs[0].y;
			
			//b2Vec2 e1 = p2 - p1;
			var e1X:Number = p2X - p1X;
			var e1Y:Number = p2Y - p1Y;
			//b2Vec2 e2 = p3 - p1;
			var e2X:Number = p3X - p1X;
			var e2Y:Number = p3Y - p1Y;
			
			//float32 D = b2Cross(e1, e2);
			var D:Number = (e1X * e2Y - e1Y * e2X);
			
			//float32 triangleArea = 0.5f * D;
			var triangleArea:Number = 0.5 * D;
			area += triangleArea;
			
			// Area weighted centroid
			//c += triangleArea * inv3 * (p1 + p2 + p3);
			cX += triangleArea * inv3 * (p1X + p2X + p3X);
			cY += triangleArea * inv3 * (p1Y + p2Y + p3Y);
		}
		
		// Centroid
		//b2Settings.b2Assert(area > Number.MIN_VALUE);
		cX *= 1.0 / area;
		cY *= 1.0 / area;
		
		// Replace return with 'out' vector
		//return c;
		out.Set(cX, cY);
	}
	
	
	
};

	
}
