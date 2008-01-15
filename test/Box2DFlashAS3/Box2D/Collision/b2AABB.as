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

package Box2D.Collision{
	
import Box2D.Collision.*
import Box2D.Common.Math.*

// A manifold for two touching convex shapes.
public class b2AABB
{
	public function IsValid():Boolean{
		//var d:b2Vec2 = b2Math.SubtractVV(maxVertex, minVertex);
		var dX:Number = maxVertex.x;
		var dY:Number = maxVertex.y;
		dX = maxVertex.x;
		dY = maxVertex.y;
		dX -= minVertex.x;
		dY -= minVertex.y;
		var valid:Boolean = dX >= 0.0 && dY >= 0.0;
		valid = valid && minVertex.IsValid() && maxVertex.IsValid();
		return valid;
	}

	public var minVertex:b2Vec2 = new b2Vec2();
	public var maxVertex:b2Vec2 = new b2Vec2();
};


}