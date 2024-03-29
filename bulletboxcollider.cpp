/* -*- mode: c++; c-basic-offset: 4; indent-tabs-mode: nil -*-

   this file is part of rcssserver3D
   Fri May 9 2003
   Copyright (C) 2002,2003 Koblenz University
   Copyright (C) 2003 RoboCup Soccer Server 3D Maintenance Group
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
#include "bulletboxcollider.h"

using namespace oxygen;
using namespace salt;


BoxColliderImp::BoxColliderImp() : ConvexColliderImp()
{
}

void BoxColliderImp::SetBoxLengths(const Vector3f& extents)
{
    //dGeomID ODEGeom = (dGeomID) geomID;
    //dGeomBoxSetLengths(
    //                   ODEGeom,
    //                   extents[0],
    //                   extents[1],
    //                   extents[2]
    //                   );
	std::cout << "RESCALED CUBE"<< std::endl;
	btBoxShape *shp = (btBoxShape *)geomID;
	shp->setLocalScaling(btVector3(btScalar(extents.x()),btScalar(extents.y()),btScalar(extents.z())));
}

long BoxColliderImp::CreateBox()
{
	btBoxShape * new_shp = new btBoxShape(btVector3(btScalar(0.5f),btScalar(0.5f),btScalar(0.5f)));
	btCollisionShape* BulletGeom = (btCollisionShape *) new_shp;
    btCollisionObject *obj = new btCollisionObject();
	obj->setUserPointer((void *)11);
	btDiscreteDynamicsWorld *wrld=lastWorld; 
	obj->setCollisionShape(BulletGeom);
	wrld->addCollisionObject(obj);

    btGeom *geom = new btGeom();
	geom->isRigidBody=false;
	geom->obj=obj;
	geom->shp=new_shp;
	geom->wrld = wrld;

   collidermap.insert(std::pair<btCollisionShape *, btGeom *>(new_shp,geom));
   geomID = new_shp;
	return (long)new_shp;
}

void BoxColliderImp::GetBoxLengths(Vector3f& extents)
{
    //dGeomID ODEGeom = (dGeomID) geomID; 
    //dVector3 lengths;
    //dGeomBoxGetLengths(ODEGeom,lengths);
    //extents[0] = lengths[0];
    //extents[1] = lengths[1];
    //extents[2] = lengths[2];
	btBoxShape *shp = (btBoxShape *)geomID;
	
	const btVector3& scale = shp->getLocalScaling();
	extents.x()=scale.getX();
	extents.y()=scale.getY();
	extents.z()=scale.getZ();
}

float BoxColliderImp::GetPointDepth(const Vector3f& pos)
{
    //dGeomID ODEGeom = (dGeomID) geomID;
    //return dGeomBoxPointDepth
    //    (ODEGeom,pos[0],pos[1],pos[2]);
    
	//TODO: implement
	std::cerr << "(BoxColliderImp) ERROR called non-implemented function GetPointDepth()" << std::endl;
	//return a value that is considered outside of the Cube
	return -0.1f;
}
