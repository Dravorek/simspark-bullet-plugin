/* -*- mode: c++; c-basic-offset: 4; indent-tabs-mode: nil -*-

   this file is part of rcssserver3D
   Fri May 9 2003
   Copyright (C) 2002,2003 Koblenz University
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
#ifndef BULLETSTATICFUNCS_H
#define BULLETSTATICFUNCS_H

#include <oxygen/physicsserver/int/physicsstaticfuncsint.h>
#include <oxygen/physicsserver/genericphysicsobjects.h>
#include <oxygen/sceneserver/basenode.h>

namespace oxygen {
class Joint;
class Collider;
class RigidBody;
}

class PhysicsStaticFuncsImp : public oxygen::PhysicsStaticFuncsInt, public oxygen::BaseNode
{
    /** See physicsserver/int/physicsobjectint.h for documentation */

public:
    PhysicsStaticFuncsImp();
    void ConvertRotationMatrix(const salt::Matrix& rot, oxygen::GenericPhysicsMatrix& matrix);
    void ConvertRotationMatrix(const oxygen::GenericPhysicsMatrix* matrix, salt::Matrix& rot) const;
	oxygen::Joint* GetJoint(oxygen::JointInt *jointID);
	bool AreConnected(oxygen::BodyInt *bodyID1, oxygen::BodyInt *bodyID2);
    bool AreConnectedExcluding(oxygen::BodyInt *bodyID1, oxygen::BodyInt *bodyID2, int joint_type);
	oxygen::RigidBody* GetBodyPointer(oxygen::BodyInt *bodyID);
	oxygen::Collider* GetColliderPointer(oxygen::ColliderInt *geomID);

	static PhysicsStaticFuncsImp *lastCreatedInstance;
};


DECLARE_CLASS(PhysicsStaticFuncsImp);

#endif //BULLETSTATICFUNCS_H
