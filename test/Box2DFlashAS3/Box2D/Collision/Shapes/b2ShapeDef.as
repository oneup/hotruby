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
import Box2D.Collision.Shapes.b2Shape;



public class b2ShapeDef
{
	public function b2ShapeDef()
	{
		type = b2Shape.e_unknownShape;
		userData = null;
		localPosition = new b2Vec2(0.0, 0.0);
		localRotation = 0.0;
		friction = 0.2;
		restitution = 0.0;
		density = 0.0;
		categoryBits = 0x0001;
		maskBits = 0xFFFF;
		groupIndex = 0;
	}

	//virtual ~b2ShapeDef() {}

	public function ComputeMass(massData:b2MassData) : void
	{	
		
		massData.center = new b2Vec2(0.0, 0.0)
		
		if (density == 0.0)
		{
			massData.mass = 0.0;
			massData.center.Set(0.0, 0.0);
			massData.I = 0.0;
		};
		
		switch (type)
		{
		case b2Shape.e_circleShape:
			{
				var circle:b2CircleDef = this as b2CircleDef;
				massData.mass = density * b2Settings.b2_pi * circle.radius * circle.radius;
				massData.center.Set(0.0, 0.0);
				massData.I = 0.5 * (massData.mass) * circle.radius * circle.radius;
			}
			break;
		
		case b2Shape.e_boxShape:
			{
				var box:b2BoxDef = this as b2BoxDef;
				massData.mass = 4.0 * density * box.extents.x * box.extents.y;
				massData.center.Set(0.0, 0.0);
				massData.I = massData.mass / 3.0 * b2Math.b2Dot(box.extents, box.extents);
			}
			break;
		
		case b2Shape.e_polyShape:
			{
				var poly:b2PolyDef = this as b2PolyDef;
				b2Shape.PolyMass(massData, poly.vertices, poly.vertexCount, density);
			}
			break;
		
		default:
			massData.mass = 0.0;
			massData.center.Set(0.0, 0.0);
			massData.I = 0.0;
			break;
		}
	}

	public var type:int;
	public var userData:* = null;
	public var localPosition:b2Vec2;
	public var localRotation:Number;
	public var friction:Number;
	public var restitution:Number;
	public var density:Number;

	// The collision category bits. Normally you would just set one bit.
	public var categoryBits:int;

	// The collision mask bits. This states the categories that this
	// shape would accept for collision.
	public var maskBits:int;

	// Collision groups allow a certain group of objects to never collide (negative)
	// or always collide (positive). Zero means no collision group. Non-zero group
	// filtering always wins against the mask bits.
	public var groupIndex:int;
};

}