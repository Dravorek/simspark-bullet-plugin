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
#ifndef BULLETWORLD_H
#define BULLETWORLD_H

#include "bulletphysicsobject.h"
#include <oxygen/physicsserver/int/worldint.h>
#include "zeitgeist\zeitgeist.h"


class WorldImp : public oxygen::WorldInt, public PhysicsObjectImp
{
/** World encapsulates a world object. It is a container for
    rigid bodies and joints. Objects in different worlds can not
    interact, for example rigid bodies from two different worlds can
    not collide. All the objects in a world exist at the same point in
    time, thus one reason to use separate worlds is to simulate
    systems at different rates.
    
    See physicsserver/int/worldint.h for documentation.
*/

public:
    WorldImp();

    void SetGravity(const salt::Vector3f& gravity);
    salt::Vector3f GetGravity() const;
    void SetERP(float erp);
    float GetERP() const;
    void SetCFM(float cfm);
    float GetCFM() const;
    void Step(float deltaTime);
    bool GetAutoDisableFlag() const;
    void SetAutoDisableFlag(bool flag);
    void SetContactSurfaceLayer(float depth);
    float GetContactSurfaceLayer() const;
    long CreateWorld();
    void DestroyWorld();
	
	btDiscreteDynamicsWorld *worldID;
};

DECLARE_CLASS(WorldImp);

#endif //BULLETWORLD_H
