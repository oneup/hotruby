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

// We use contact ids to facilitate warm starting.
public class Features
{
	//
	public function set referenceFace(value:int) : void{
		_referenceFace = value;
		_m_id._key = (_m_id._key & 0xffffff00) | (_referenceFace & 0x000000ff)
	}
	public function get referenceFace():int{
		return _referenceFace;
	}
	public var _referenceFace:int;
	//
	public function set incidentEdge(value:int) : void{
		_incidentEdge = value;
		_m_id._key = (_m_id._key & 0xffff00ff) | ((_incidentEdge << 8) & 0x0000ff00)
	}
	public function get incidentEdge():int{
		return _incidentEdge;
	}
	public var _incidentEdge:int;
	//
	public function set incidentVertex(value:int) : void{
		_incidentVertex = value;
		_m_id._key = (_m_id._key & 0xff00ffff) | ((_incidentVertex << 16) & 0x00ff0000)
	}
	public function get incidentVertex():int{
		return _incidentVertex;
	}
	public var _incidentVertex:int;
	//
	public function set flip(value:int) : void{
		_flip = value;
		_m_id._key = (_m_id._key & 0x00ffffff) | ((_flip << 24) & 0xff000000)
	}
	public function get flip():int{
		return _flip;
	}
	public var _flip:int;
	public var _m_id:b2ContactID;
};


}