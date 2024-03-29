/* -*- mode: c++; c-basic-offset: 4; indent-tabs-mode: nil -*-

   this file is part of rcssserver3D
   Fri May 9 2003
   Copyright (C) 2011 Koblenz University
   Copyright (C) 2003 RoboCup Soccer Server 3D Maintenance Group
   $Id$
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
#ifndef BULLETPHYSICSOBJECT_H
#define BULLETPHYSICSOBJECT_H

#include "btBulletDynamicsCommon.h"

#include <oxygen/physicsserver/int/physicsobjectint.h>
#include <oxygen/physicsserver/genericphysicsobjects.h>
#include <oxygen/sceneserver/basenode.h>

struct btGeom{
	btGeom(): isRigidBody(true){}
    btCollisionObject *obj; //btRigidBody
    btCollisionShape *shp;
    btDiscreteDynamicsWorld *wrld;
	bool isRigidBody;
};

enum jointtype{
	JT_FIXED,JT_HINGE,JT_HINGE2,JT_SLIDER,JT_UNIVERSAL,JT_OTHER
};
struct btJointWrapper{
	btJointWrapper(): joint(NULL), type(JT_OTHER), world(NULL), added(false){}
	btTypedConstraint *joint;
	jointtype type; 
	btDiscreteDynamicsWorld *world;
	bool added;
};

typedef btGeom* btGeomID;

#include <map>
extern std::map<btCollisionShape *,btGeom *> collidermap;
extern btDiscreteDynamicsWorld *lastWorld;
extern std::multimap<long,void*> spaces;

class PhysicsObjectImp :  public oxygen::PhysicsObjectInt, public oxygen::BaseNode 
{
    /** See physicsserver/int/physicsobjectint.h for documentation */

public:
    PhysicsObjectImp();
    void ConvertRotationMatrix(const salt::Matrix& rot, oxygen::GenericPhysicsMatrix& matrix);
    void ConvertRotationMatrix(const oxygen::GenericPhysicsMatrix* matrix, salt::Matrix& rot) const;

	virtual oxygen::ColliderInt *UpcastToCollider();
	virtual oxygen::SpaceInt *UpcastToSpace();
};

DECLARE_CLASS(PhysicsObjectImp);

#endif //BULLETPHYSICSOBJECT_H
