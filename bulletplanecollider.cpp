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

#include "btBulletCollisionCommon.h"
#include "bulletplanecollider.h"

using namespace oxygen;
using namespace salt;
#include <map>
extern std::map<btCollisionShape *,btGeom *> collidermap;
PlaneColliderImp::PlaneColliderImp() : ColliderImp()
{
}

void PlaneColliderImp::SetPlaneParams(float a, float b, float c, float d, long geomID)
{
	btStaticPlaneShape *plane = (btStaticPlaneShape *)geomID;
	btStaticPlaneShape *new_plane = new btStaticPlaneShape( btVector3(btScalar(a),btScalar(b),btScalar(c)), btScalar(d) );
	
	*plane = std::move(*new_plane);
	auto it = collidermap.find(plane);
	if(it!=collidermap.end()){
		it->second->wrld->removeRigidBody(it->second->obj);
		it->second->obj->setCollisionShape(plane);
		it->second->wrld->addRigidBody(it->second->obj);
	}
	//memcpy(plane,new_plane,sizeof(new_plane));
	//delete new_plane;
	
	//:TODO: DO NOT COMMIT WITH THIS BROKEN CODE
	//:HACK: seriously? copying dynamically allocated objects?
    //dGeomID ODEGeom = (dGeomID) geomID;
    //dGeomPlaneSetParams(ODEGeom, a, b, c, d);
}

long PlaneColliderImp::CreatePlane()
{
   // // a plane with normal pointing up, going through the origin
   btStaticPlaneShape *new_plane = new btStaticPlaneShape( btVector3(btScalar(0.0f),btScalar(1.0f),btScalar(0.0f)), btScalar(0.0f) );
	return (long)new_plane;
}

void PlaneColliderImp::SetParams(const salt::Vector3f& pos, salt::Vector3f normal, long geomID)
{
    //dGeomID ODEGeom = (dGeomID) geomID;
    normal.Normalize();
    float d = pos.Dot(normal);
    SetPlaneParams(normal.x(), normal.y(), normal.z(), d,geomID);
}

float PlaneColliderImp::GetPointDepth(const Vector3f& pos, long geomID)
{
    //dGeomID ODEGeom = (dGeomID) geomID;
    //return dGeomPlanePointDepth
    //    (ODEGeom,pos[0],pos[1],pos[2]);
    //:TODO: check if bullet doesn't have a comaprable function

	btStaticPlaneShape *plane = (btStaticPlaneShape *)geomID;
	const btVector3& normal = plane->getPlaneNormal();
	const btScalar& constant = plane->getPlaneConstant();
	return constant - normal.getX()*pos.x() - normal.getY()*pos.y() - normal.getZ()*pos.z();
}
