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

#include "bulletspace.h"
#include <oxygen/physicsserver/collider.h>
#include <oxygen/physicsserver/space.h>
#include <map>

using namespace oxygen;



void SpaceImp::collisionNearCallback(void* data, long obj1, long obj2)
{
    //handled by the narrowphase collision handler which gets called in
    //stepWorld();
	std::cerr << "(SpaceImp) ERROR called unimplemented method CreateSliderJoint(" << std::endl;
}

SpaceImp::SpaceImp() : PhysicsObjectImp(), innercollision(true), spaceID(0)
{
}

void SpaceImp::Collide(SpaceInt *space, Space* callee)
{
    //gets handled by the broadphase collision handler when
    //stepWorld() gets called
}

void SpaceImp::Collide2(PhysicsObjectInt *obj1, PhysicsObjectInt *obj2, Space* callee)
{
    //gets handled by the broadphase collision handler when
    //stepWorld() gets called
	std::cerr << "(SpaceImp) ERROR called unimplemented method Collide2(" << std::endl;
}

SpaceInt *SpaceImp::GetParentSpaceID()
{
    //not used by the Bullet implementation
    //dGeomID SpaceImp = (dGeomID) spaceID;
    //dSpaceID parentSpace = dGeomGetSpace(SpaceImp);
    //return (long) parentSpace;
    
	std::cerr << "(SpaceImp) DEBUG getParentSpace() "<< spaceID << std::endl; 
    
	if(spaceID==1)
		return 0l;

	for(auto it=spaces.begin(); it !=spaces.end();it++){
		if((long)((*it).second)== spaceID)
			return (SpaceInt *)it->first;
	}
	std::cerr << "(SpaceImp) DEBUG top level space found"<< spaceID << std::endl; 
	return (SpaceInt *)1l;
}

PhysicsObjectInt *SpaceImp::CreateContactGroup()
{
    //not used by the bullet implementation

    // create a joint group for the contacts
    //dJointGroupID ODEContactGroup = dJointGroupCreate(0);

    //return (long) ODEContactGroup;
	std::cerr << "(SpaceImp) ERROR called unimplemented function CreateContactGroup()" << std::endl; 
    return reinterpret_cast<PhysicsObjectInt *>(1l);
}

void SpaceImp::PostPhysicsUpdateInternal(PhysicsObjectInt *contactGroup)
{
    //not used by the bullet implementation

    //dJointGroupID ODEContactGroup = (dJointGroupID) contactGroup;
    // remove all contact joints
    //dJointGroupEmpty(ODEContactGroup);
}

long SpaceImp::CreateSpace(SpaceInt *spaceID2){
    //not supported by the bullet implementation
    
	static int counter = 2;
	int newSpace = counter++;
	spaces.insert( std::pair<long,void*>(spaceID2?(int)spaceID2:1,(void*)this));

	//dSpaceID SpaceImp = (dSpaceID) spaceID;
    //dSpaceID CreatedSpace = dHashSpaceCreate(SpaceImp);
    //return (long) CreatedSpace;
	std::cerr << "(SpaceImp) DEBUG created space:" << newSpace << std::endl; 
	spaceID = (long)this;//newSpace;
    return (long)this;
}

void SpaceImp::DestroySpace(PhysicsObjectInt *contactGroup)
{
    //not supported by bullet
	long parent = (long)GetParentSpaceID();
	if(parent==0) return;

	auto it = spaces.begin();
	
	//delete the parent entry
	while(it!=spaces.end())
	{
		if((int)it->second == spaceID)
		{
			spaces.erase(it);
			break;
		}
	}
	//find all the children and save them
	auto range = spaces.equal_range(spaceID);
	std::vector<void *>vec;
	for(auto iter= range.first;iter!=range.second;iter++)
	{
		vec.push_back(it->second);
	}
	//delete the children entries
	spaces.erase(range.first,range.second);
	std::cerr << "(SpaceImp) DEBUG deleted space:" << spaceID
		      << "  contact group:" << contactGroup << std::endl; 

}

bool SpaceImp::ObjectIsSpace(PhysicsObjectInt *objectID){
    //not supported by bullet
	bool isSpace = spaces.find((int)objectID)!=spaces.end();
	std::cerr << "(SpaceImp) DEBUG isSpace(" << objectID << ")="
		      << isSpace << std::endl;
	return isSpace;
}

BodyInt *SpaceImp::FetchBody(ColliderInt *geomID){
    //not necessary, as only used by the unsupported Space::Collide()
    std::cerr << "(SpaceImp) ERROR called unimplemented function FetchBody()" << std::endl; 
    return NULL;
}

SpaceInt *SpaceImp::FetchSpace(PhysicsObjectInt *geomID){
    //not necessary, as only used by the unsupported Space::Collide()
    std::cerr << "(SpaceImp) ERROR called unimplemented function FetchSpace()" << std::endl; 
    return NULL;
}

bool SpaceImp::AreConnectedWithJoint(const BodyInt *bodyID1,const  BodyInt *bodyID2){
    //only used by Space::Collide(); so not used in bullet implementation
    std::cerr << "(SpaceImp) ERROR called unimplemented function AreConnectedWithJoint()" << std::endl; 
    return true;
}

void SpaceImp::CollideInternal(boost::shared_ptr<Collider> collider, 
                              boost::shared_ptr<Collider> collidee,
                              ColliderInt *geomID1, ColliderInt *geomID2)
{
    //not used by the bullet immplementation the world object handles all collisions
}

void SpaceImp::DisableInnerCollision(bool disable){
	this->innercollision = !disable;
	//:TODO: search through all rigidbodys in this space and re-set their collisionflags
	auto range = spaces.equal_range(spaceID);
	std::vector<void *>vec;
	for(auto iter= range.first;iter!=range.second;iter++)
	{
		void *object = iter->second;

		auto it = collidermap.find((btCollisionShape *) object);
		//part of CollisionObject?
		if(it!=collidermap.end())
		{
			btGeom *temp = it->second;
			if(temp->obj && temp->wrld)
			{
				if(temp->isRigidBody)
				{
					temp->wrld->removeRigidBody(static_cast<btRigidBody*>(temp->obj));
					temp->wrld->addRigidBody(static_cast<btRigidBody*>(temp->obj));
					((btRigidBody *)temp->obj)->activate(true);
					((btRigidBody *)temp->obj)->setActivationState(DISABLE_DEACTIVATION);

				}
				else
				{
					temp->wrld->removeCollisionObject(temp->obj);
					temp->wrld->addCollisionObject(temp->obj);
				}
			}
		}
	}
}

bool SpaceImp::hasInnerCollision(){
	return !this->innercollision;
}
