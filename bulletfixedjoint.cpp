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

#include "bulletfixedjoint.h"
#include "bulletbody.h"
#include "bulletworld.h"

using namespace oxygen;
using namespace boost;

FixedJointImp::FixedJointImp() : Generic6DOFJointImp()
{
	std::cerr << "(FixedJointImp) ERROR called unimplemented constructor" << std::endl;
}

long FixedJointImp::CreateFixedJoint(WorldInt *world)
{
	//btDiscreteDynamicsWorld *wrld = reinterpret_cast<btDiscreteDynamicsWorld *>(world);
    
	btJointWrapper *wrp = new btJointWrapper();
	btRigidBody *rbA = new btRigidBody(0.0,0,0);
	rbA->activate(true);
	rbA->setActivationState(DISABLE_DEACTIVATION);
	rbA->setUserPointer((void *)99);
	btRigidBody *rbB = new btRigidBody(0.0,0,0);
	rbB->activate(true);
	rbB->setActivationState(DISABLE_DEACTIVATION);
	rbB->setUserPointer((void *)99);
	wrp->joint = new btGeneric6DofConstraint(*rbA,*rbB,btTransform(),btTransform(),true);
	wrp->type = JT_FIXED;
	wrp->world = static_cast<WorldImp *>(world)->worldID;
	jointID = wrp;
	return reinterpret_cast<long>(wrp);
	
	
	//dWorldID ODEworld = (dWorldID) world;
    //dJointID ODEJoint = dJointCreateFixed(ODEworld, 0);
    //return (long) ODEJoint;
	//std::cerr << "(FixedJointImp) ERROR called unimplemented method: CreateFixedJoint(" << std::endl;
    //return 0l;
}

void FixedJointImp::SetFixed()
{
    //dJointID ODEJoint = (dJointID) jointID;
    //dJointSetFixed(ODEJoint);
	std::cerr << "(FixedJointImp) ERROR called unimplemented method: SetFixed(" << std::endl;
}

void FixedJointImp::Attach(BodyInt *bodyID1, BodyInt *bodyID2){
	btJointWrapper *wrp = new btJointWrapper();
	btGeom * rbA = (btGeom *)static_cast<BodyImp *>(bodyID1)->btID;
	btGeom * rbB = (btGeom *)static_cast<BodyImp *>(bodyID2)->btID;
	
	btTransform trans1 = btTransform::getIdentity();
	btMatrix3x3 rotmat = rbA->obj->getWorldTransform().getBasis().transpose();
	rotmat = rotmat * rbB->obj->getWorldTransform().getBasis();
	trans1.setBasis(rotmat);
	btVector3 anchor(0.0,0.0,0.0);
	btVector3 axis1(0.0,1.0,0.0);
	btVector3 axis2(1.0,0.0,0.0);
	if(jointID)
	{
	if(jointID->joint){
		if(jointID->added)lastWorld->removeConstraint(jointID->joint);
		btRigidBody *rbaa = &(jointID->joint->getRigidBodyA());
		btRigidBody *rbbb = &(jointID->joint->getRigidBodyB());
		delete jointID->joint;
		delete rbaa;
		delete rbbb;
	}
	delete jointID;
	}
	//new btGeneric6DofConstraint(
	btGeneric6DofConstraint * joint = new btGeneric6DofConstraint(*((btRigidBody *)rbA->obj),*((btRigidBody *)rbB->obj),btTransform::getIdentity(),trans1,false);
	//btUniversalConstraint * joint= new btUniversalConstraint(*((btRigidBody *)rbA->obj),*((btRigidBody *)rbB->obj),anchor,axis1,axis2);
	joint->setLinearLowerLimit(btVector3(0.0,0.0,0.0));
	joint->setLinearUpperLimit(btVector3(0.0,0.0,0.0));
	joint->setAngularLowerLimit(btVector3(0.0,0.0,0.0));
	joint->setAngularUpperLimit(btVector3(0.0,0.0,0.0));
	wrp->joint = joint;
	lastWorld->addConstraint(joint,true);
	wrp->added = true;

	jointID = wrp;
	//{
	//		btBoxShape *box1;
	//		btBoxShape *box2;
	//		btRigidBody *rb1;
	//		btRigidBody *rb2;
	//		btTransform trans;
	//		btTransform trans2;
	//		btVector3 inertia1;
	//		btVector3 inertia2;

	//		btHingeConstraint *hingejoint;

	//		trans = rbA->obj->getWorldTransform();
	//		trans.setOrigin(trans.getOrigin()+btVector3(0.0,0.0,4.0));

	//		trans2 = trans;
	//		trans2.setOrigin(trans.getOrigin()+btVector3(0.0,2.4,0.0));

	//		box1 = new btBoxShape(btVector3( 1.0,1.0,1.0));
	//		box2 = new btBoxShape(btVector3( 1.0,1.0,1.0));
	//		
	//		box1->calculateLocalInertia(1.0,inertia1);
	//		box2->calculateLocalInertia(1.0,inertia2);

	//		rb1 = new btRigidBody(1.0,new btDefaultMotionState(trans),box1,inertia1);
	//		rb2 = new btRigidBody(1.0,new btDefaultMotionState(trans2),box2,inertia2);

	//		hingejoint = new btHingeConstraint(*rb1,btTransform::getIdentity());
	//		/*hingejoint = new btHingeConstraint(*rb1,
	//										   *rb2,
	//										   btVector3(0.0,1.2,0.0),
	//										   btVector3(0.0,-1.2,0.0),
	//										   btVector3(1.0,0.0,0.0),
	//										   btVector3(-1.0,0.0,0.0),
	//										   false);
	//		*/
	//		lastWorld->addRigidBody(rb1);
	//		lastWorld->addRigidBody(rb2);
	//		lastWorld->addConstraint(hingejoint,true);
	//	}
}
