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

#include "bulletphysicsstaticfuncs.h"

#include "btBulletDynamicsCommon.h"

#include "bulletjoint.h"
#include "bulletcollider.h"
#include "bulletrigidbody.h"

PhysicsStaticFuncsImp *PhysicsStaticFuncsImp::lastCreatedInstance = nullptr;

PhysicsStaticFuncsImp::PhysicsStaticFuncsImp()
{
	lastCreatedInstance=this;
}

void PhysicsStaticFuncsImp::ConvertRotationMatrix(const salt::Matrix& rot, oxygen::GenericPhysicsMatrix& matrix)
{
	btMatrix3x3& btMatrix = (btMatrix3x3&) matrix;
    
    btMatrix[0][0] = rot.m[0];
    btMatrix[1][0] = rot.m[1];
    btMatrix[2][0] = rot.m[2];
    btMatrix[0][1] = rot.m[4];
    btMatrix[1][1] = rot.m[5];
    btMatrix[2][1] = rot.m[6];
    btMatrix[0][2] = rot.m[8];
    btMatrix[1][2] = rot.m[9];
    btMatrix[2][2] = rot.m[10];
}


void PhysicsStaticFuncsImp::ConvertRotationMatrix(const oxygen::GenericPhysicsMatrix* matrix, salt::Matrix& rot) const
{
	btMatrix3x3& btMatrix = (btMatrix3x3&) matrix;
    
   rot.m[0]   = (float)btMatrix[0][0];
   rot.m[1]   = (float)btMatrix[1][0];
   rot.m[2]   = (float)btMatrix[2][0];
   rot.m[3]   = 0.0f;
   rot.m[4]   = (float)btMatrix[0][1];
   rot.m[5]   = (float)btMatrix[1][1];
   rot.m[6]   = (float)btMatrix[2][1];
   rot.m[7]   = 0.0f;
   rot.m[8]   = (float)btMatrix[0][2];
   rot.m[9]   = (float)btMatrix[1][2];
   rot.m[10]  = (float)btMatrix[2][2];
   rot.m[11]  = 0.0f;
   rot.m[12]  = 0.0f;
   rot.m[13]  = 0.0f;
   rot.m[14]  = 0.0f;
   rot.m[15]  = 1.0f;
}

oxygen::Joint* PhysicsStaticFuncsImp::GetJoint(oxygen::JointInt *jointID)
{
	return reinterpret_cast<oxygen::Joint *>(static_cast<JointImp *>(jointID)->userPointer);
}

bool PhysicsStaticFuncsImp::AreConnected(oxygen::BodyInt *bodyID1, oxygen::BodyInt *bodyID2)
{
	    //dBodyID ODEBody1 = (dBodyID) bodyID1;
    //dBodyID ODEBody2 = (dBodyID) bodyID2;
    //return dAreConnected(ODEBody1, ODEBody2) == 1;
	//std::cerr << "(JointImp) WARNING called unfinished (inefficient) method AreConnected(" << std::endl;
	//:TODO: find out if this includes contact constraints in ODE?
	// this: http://opende.sourceforge.net/wiki/index.php/Manual_(Joint_Types_and_Functions)
	//would suggest that they are, what now?
	BodyImp *imp1 = static_cast<BodyImp*>(bodyID1);
	BodyImp *imp2 = static_cast<BodyImp*>(bodyID2);

	int numConstraints = lastWorld->getNumConstraints();
	for(int i = 0; i<numConstraints;i++){
		btTypedConstraint *joint = lastWorld->getConstraint(i);
		if( &(joint->getRigidBodyA())==imp1->btID->obj)
		{
			if(&(joint->getRigidBodyB())==imp2->btID->obj)
				return true;
		}
		else if(&(joint->getRigidBodyB())==imp1->btID->obj)
		{
			if(&(joint->getRigidBodyA())==imp2->btID->obj)
				return true;
		}
	}
    return false;
}

bool PhysicsStaticFuncsImp::AreConnectedExcluding(oxygen::BodyInt *bodyID1, oxygen::BodyInt *bodyID2, int joint_type)
{
	    //dBodyID ODEBody1 = (dBodyID) bodyID1;
    //dBodyID ODEBody2 = (dBodyID) bodyID2;
    //return dAreConnectedExcluding(ODEBody1, ODEBody2, joint_type) == 1;
	//std::cerr << "(JointImp) ERROR called unfinished method AreConnectedExcluding(" << std::endl;

	BodyImp *imp1 = static_cast<BodyImp*>(bodyID1);
	BodyImp *imp2 = static_cast<BodyImp*>(bodyID2);

	int numConstraints = lastWorld->getNumConstraints();
	for(int i = 0; i<numConstraints;i++){
		btTypedConstraint *joint = lastWorld->getConstraint(i);
		if( &(joint->getRigidBodyA())==imp1->btID->obj)
		{
			if(&(joint->getRigidBodyB())==imp2->btID->obj)
				if(joint->getConstraintType()!=joint_type)
					return true;
		}
		else if(&(joint->getRigidBodyB())==imp1->btID->obj)
		{
			if(&(joint->getRigidBodyA())==imp2->btID->obj)
				if(joint->getConstraintType()!=joint_type)
					return true;
		}
	}
	
	return false;
}

oxygen::RigidBody* PhysicsStaticFuncsImp::GetBodyPointer(oxygen::BodyInt *bodyID)
{
	return reinterpret_cast<oxygen::RigidBody *>(static_cast<RigidBodyImp *>(bodyID)->userPointer);

}

oxygen::Collider* PhysicsStaticFuncsImp::GetColliderPointer(oxygen::ColliderInt *geomID)
{
	return reinterpret_cast<oxygen::Collider *>(static_cast<ColliderImp *>(geomID)->userPointer);
}


