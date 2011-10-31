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

using namespace oxygen;

void SpaceImp::collisionNearCallback(void* data, long *obj1, long *obj2)
{
    //handled by the narrowphase collision handler which gets called in
    //stepWorld();
    
    //Space* space = (Space*) data;
    //space->HandleCollide((long) obj1, (long) obj2);
}

SpaceImp::SpaceImp() : PhysicsObjectImp()
{
}

void SpaceImp::Collide(long space, Space* callee)
{
    //gets handled by the broadphase collision handler when
    //stepWorld() gets called

    //dSpaceID SpaceImp = (dSpaceID) space;
    //dSpaceCollide(SpaceImp, callee, collisionNearCallback);
}

void SpaceImp::Collide2(long obj1, long obj2, Space* callee)
{
    //gets handled by the broadphase collision handler when
    //stepWorld() gets called

    //dGeomID ODEObj1 = (dGeomID) obj1;
    //dGeomID ODEObj2 = (dGeomID) obj2;
    //dSpaceCollide2(ODEObj1, ODEObj2, callee, &collisionNearCallback);
}

long SpaceImp::GetParentSpaceID(long spaceID)
{
    //not used by the Bullet implementation

    //dGeomID SpaceImp = (dGeomID) spaceID;
    //dSpaceID parentSpace = dGeomGetSpace(SpaceImp);
    //return (long) parentSpace;
    return 0l;
}

long SpaceImp::CreateContactGroup()
{
    //not used by the bullet implementation

    // create a joint group for the contacts
    //dJointGroupID ODEContactGroup = dJointGroupCreate(0);

    //return (long) ODEContactGroup;
    return 0l;
}

void SpaceImp::PostPhysicsUpdateInternal(long contactGroup)
{
    //not used by the bullet implementation

    //dJointGroupID ODEContactGroup = (dJointGroupID) contactGroup;
    // remove all contact joints
    //dJointGroupEmpty(ODEContactGroup);
}

long SpaceImp::CreateSpace(long spaceID){
    //not supported by the bullet implementation
    
    //dSpaceID SpaceImp = (dSpaceID) spaceID;
    //dSpaceID CreatedSpace = dHashSpaceCreate(SpaceImp);
    //return (long) CreatedSpace;

    return 0l;
}

void SpaceImp::DestroySpace(long contactGroup, long spaceID)
{
    //not supported by bullet
}

bool SpaceImp::ObjectIsSpace(long objectID){
    //not supported by bullet
    return false;
}

long SpaceImp::FetchBody(long geomID){
    //not necessary, as only used by the unsupported Space::Collide()
    return 0l;
}

long SpaceImp::FetchSpace(long geomID){
    //not necessary, as only used by the unsupported Space::Collide()
    return 0l;
}

bool SpaceImp::AreConnectedWithJoint(long bodyID1, long bodyID2){
    //only used by Space::Collide(); so not used in bullet implementation
    return true;
}

void SpaceImp::CollideInternal(boost::shared_ptr<Collider> collider, 
                              boost::shared_ptr<Collider> collidee,
                              long geomID1, long geomID2)
{
    //not used by the bullet immplementation the world object handles all collisions
}
