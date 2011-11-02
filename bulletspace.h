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
#ifndef BULLETSPACE_H
#define BULLETSPACE_H

#include "bulletphysicsobject.h"
#include <oxygen/physicsserver/int/spaceint.h>

class SpaceImp : public oxygen::SpaceInt, public PhysicsObjectImp
{

/** The Space concept is currently not supported in Bullet.
    Most of the functions here are dummys and don't do anything.
    The collisions are handled during the world stepping.
    
    See physicsserver/int/spaceint.h for documentation.
*/

public:
    SpaceImp();
    long CreateSpace(long spaceID);
    void DestroySpace(long contactGroup, long spaceID);
    long GetParentSpaceID(long spaceID);
    long CreateContactGroup();
    void PostPhysicsUpdateInternal(long contactGroup);
    void Collide(long spaceID, oxygen::Space* callee);
    void Collide2(long obj1, long obj2, oxygen::Space* callee);
    bool ObjectIsSpace(long objectID);
    long FetchBody(long geomID);
    long FetchSpace(long geomID);
    bool AreConnectedWithJoint(long bodyID1, long bodyID2);
    void CollideInternal(boost::shared_ptr<oxygen::Collider> collider, 
                        boost::shared_ptr<oxygen::Collider> collidee,
                        long geomID1, long geomID2);
    
private:
    static void collisionNearCallback(void* data, long obj1, long obj2);
};

DECLARE_CLASS(SpaceImp);

#endif //ODESPACE_H
