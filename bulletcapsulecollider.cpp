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

void CapsuleColliderImp::SetParams(float radius, float length)
{
    btCapsuleShape *shp = (btCapsuleShape *) geomID;
	btCapsuleShape *new_shp = new btCapsuleShapeZ(radius,length);
	//:HACK:
	//:TODO: clean this up, this is no way to deal with the immutable problem
	*shp = std::move(*new_shp);
	auto it = collidermap.find(shp);
	if(it!=collidermap.end()){
		if(it->second->isRigidBody)
			it->second->wrld->removeRigidBody(((btRigidBody *)it->second->obj));
		else
			it->second->wrld->removeCollisionObject(it->second->obj);
		it->second->obj->setCollisionShape(shp);
		if(it->second->isRigidBody)
		{
			it->second->wrld->addRigidBody(((btRigidBody *)it->second->obj));
			static_cast<btRigidBody *>(it->second->obj)->activate(true);
			((btRigidBody *)it->second->obj)->setActivationState(DISABLE_DEACTIVATION);
		}
		else
			it->second->wrld->addCollisionObject(it->second->obj);		
	}
	delete new_shp;
}

void CapsuleColliderImp::SetRadius(float radius)
{
    btCapsuleShape *shp = (btCapsuleShape *) geomID;
	btCapsuleShape *new_shp = new btCapsuleShapeZ(radius,shp->getHalfHeight()*2.0);
	//:HACK:
	//:TODO: clean this up, this is no way to deal with the immutable problem
	*shp = std::move(*new_shp);
	auto it = collidermap.find(shp);
	if(it!=collidermap.end()){
		if(it->second->isRigidBody)
			it->second->wrld->removeRigidBody(((btRigidBody *)it->second->obj));
		else
			it->second->wrld->removeCollisionObject(it->second->obj);
		it->second->obj->setCollisionShape(shp);
		if(it->second->isRigidBody)
		{	
			it->second->wrld->addRigidBody(((btRigidBody *)it->second->obj));
			static_cast<btRigidBody *>(it->second->obj)->activate(true);
			((btRigidBody *)it->second->obj)->setActivationState(DISABLE_DEACTIVATION);
		}
		else
			it->second->wrld->addCollisionObject(it->second->obj);		

	}
	delete new_shp;
}

void CapsuleColliderImp::SetLength(float length)
{
    btCapsuleShape *shp = (btCapsuleShape *) geomID;
	btCapsuleShape *new_shp = new btCapsuleShapeZ(shp->getRadius(),length);
	//:HACK:
	//:TODO: clean this up, this is no way to deal with the immutable problem
	*shp = std::move(*new_shp);
	auto it = collidermap.find(shp);
	if(it!=collidermap.end()){
		if(it->second->isRigidBody)
			it->second->wrld->removeRigidBody(((btRigidBody *)it->second->obj));
		else
			it->second->wrld->removeCollisionObject(it->second->obj);
		it->second->obj->setCollisionShape(shp);
		if(it->second->isRigidBody)
		{
			it->second->wrld->addRigidBody(((btRigidBody *)it->second->obj));
			static_cast<btRigidBody *>(it->second->obj)->activate(true);
			((btRigidBody *)it->second->obj)->setActivationState(DISABLE_DEACTIVATION);
		}
		else
			it->second->wrld->addCollisionObject(it->second->obj);		
	}
	delete new_shp;
}

void CapsuleColliderImp::GetParams(float& radius, float& length)
{
    btCapsuleShape *shp = (btCapsuleShape *) geomID;
	radius = shp->getRadius();
	length = shp->getHalfHeight()*2.0;
}

float CapsuleColliderImp::GetRadius()
{
    btCapsuleShape *shp = (btCapsuleShape *) geomID;
	return shp->getRadius();
}

float CapsuleColliderImp::GetLength()
{
    btCapsuleShape *shp = (btCapsuleShape *) geomID;
	return shp->getHalfHeight()*2.0f;
}

long CapsuleColliderImp::CreateCapsule()
{
	btCapsuleShapeZ * new_shp = new btCapsuleShapeZ(1.0f,1.0f);
	btCollisionShape* BulletGeom = (btCollisionShape *) new_shp;
    btCollisionObject *obj = new btCollisionObject();
	obj->setUserPointer((void *)14);
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

float CapsuleColliderImp::GetPointDepth(const Vector3f& pos)
{
    //dGeomID ODEGeom = (dGeomID) geomID;
    //return dGeomCapsulePointDepth
    //    (ODEGeom,pos[0],pos[1],pos[2]);
	//std::cerr <<" (SphereColliderImp) ERROR called unimplemented method GetPointDepth()" << std::endl;

	//:TODO: implement
	//returning value that's considered to be on the outside, so as not to generate unnessesary collision events
	return -0.1f;
}
