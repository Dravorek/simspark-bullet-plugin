/* -*- mode: c++; c-basic-offset: 4; indent-tabs-mode: nil -*-

   this file is part of rcssserver3D
   Fri May 9 2003
   Copyright (C) 2003 Koblenz University
   $Id$

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; version 2 of the License.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#include "bulletcapsulecollider.h"

using namespace oxygen;
using namespace salt;

CapsuleColliderImp::CapsuleColliderImp() : ConvexColliderImp()
{
}

void CapsuleColliderImp::SetParams(float radius, float length, long geomID)
{
    btCapsuleShape *shp = (btCapsuleShape *) geomID;
	btCapsuleShape *new_shp = new btCapsuleShape(radius,length);
	//:HACK:
	//:TODO: clean this up, this is no way to deal with the immutable problem
	memcpy(shp,new_shp,sizeof(new_shp));
	delete new_shp;
}

void CapsuleColliderImp::SetRadius(float radius, long geomID)
{
    btCapsuleShape *shp = (btCapsuleShape *) geomID;
	btCapsuleShape *new_shp = new btCapsuleShape(radius,shp->getHalfHeight()*2.0);
	//:HACK:
	//:TODO: clean this up, this is no way to deal with the immutable problem
	memcpy(shp,new_shp,sizeof(new_shp));
	delete new_shp;
}

void CapsuleColliderImp::SetLength(float length, long geomID)
{
    btCapsuleShape *shp = (btCapsuleShape *) geomID;
	btCapsuleShape *new_shp = new btCapsuleShape(shp->getRadius(),length);
	//:HACK:
	//:TODO: clean this up, this is no way to deal with the immutable problem
	memcpy(shp,new_shp,sizeof(new_shp));
	delete new_shp;
}

void CapsuleColliderImp::GetParams(float& radius, float& length, long geomID)
{
    btCapsuleShape *shp = (btCapsuleShape *) geomID;
	radius = shp->getRadius();
	length = shp->getHalfHeight()*2.0;
}

float CapsuleColliderImp::GetRadius(long geomID)
{
    btCapsuleShape *shp = (btCapsuleShape *) geomID;
	return shp->getRadius();
}

float CapsuleColliderImp::GetLength(long geomID)
{
    btCapsuleShape *shp = (btCapsuleShape *) geomID;
	return shp->getHalfHeight()*2.0f;
}

long CapsuleColliderImp::CreateCapsule()
{
	return (long)new btCapsuleShape(1.0f,1.0f);
}

float CapsuleColliderImp::GetPointDepth(const Vector3f& pos, long geomID)
{
    //dGeomID ODEGeom = (dGeomID) geomID;
    //return dGeomCapsulePointDepth
    //    (ODEGeom,pos[0],pos[1],pos[2]);
	std::cerr <<" (SphereColliderImp) ERROR called unimplemented method GetPointDepth()"
		<< std::endl;

	//:TODO: implement
	//returning value that's considered to be on the outside, so as not to generate unnessesary collision events
	return -0.1f;
}
