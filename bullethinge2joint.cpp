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

#include "bullethinge2joint.h"
#include "bulletjoint.h"
#include "bulletbody.h"
#include "bulletworld.h"

using namespace oxygen;
using namespace boost;
using namespace salt;

Hinge2JointImp::Hinge2JointImp() : Generic6DOFJointImp()
{
	std::cerr << "(Hinge2JointImp) ERROR called unimplemented constructor" << std::endl;
}

long Hinge2JointImp::CreateHinge2Joint(WorldInt *worldID)
{
    //dWorldID ODEWorld = (dWorldID) worldID;
    //dJointID ODEJoint = dJointCreateHinge2(ODEWorld, 0);
    //return (long) ODEJoint;
	btJointWrapper *wrp = new btJointWrapper();
	btRigidBody *rbA = new btRigidBody(0.0,0,0);
	rbA->activate(true);
	rbA->setActivationState(DISABLE_DEACTIVATION);
	rbA->setUserPointer((void *)99);
	btRigidBody *rbB = new btRigidBody(0.0,0,0);
	rbB->activate(true);
	rbB->setActivationState(DISABLE_DEACTIVATION);
	rbB->setUserPointer((void *)99);
	wrp->joint = new btHinge2Constraint(*rbA,*rbB,btVector3(0,0,0),btVector3(1,0,0),btVector3(1,0,0));
	wrp->type = JT_HINGE2;
	wrp->world = static_cast<WorldImp *>(worldID)->worldID;

	jointID = wrp;
	return reinterpret_cast<long>(wrp);
	
	std::cerr << "(Hinge2JointImp) ERROR called unimplemented method: CreateHinge2Joint(" << std::endl;
    return (long)wrp;
}

void Hinge2JointImp::SetAnchor(const Vector3f& gAnchor,
                            const Vector3f& up,
                            const Vector3f& right)
{
    //dJointID ODEJoint = (dJointID) jointID;
    //dJointSetHinge2Anchor (ODEJoint, gAnchor[0], gAnchor[1], gAnchor[2]);
    //dJointSetHinge2Axis1(ODEJoint,up[0],up[1],up[2]);
    //dJointSetHinge2Axis2(ODEJoint,right[0],right[1],right[2]);
	std::cerr << "(Hinge2JointImp) ERROR called unimplemented method: SetAnchor(" << std::endl;
}

Vector3f Hinge2JointImp::GetAnchor1()
{
    //dJointID ODEJoint = (dJointID) jointID;
    //dReal anchor[3];
    //dJointGetHinge2Anchor (ODEJoint, anchor);
    //Vector3f pos = Vector3f(anchor[0],anchor[1],anchor[2]);
    //return pos;
	std::cerr << "(Hinge2JointImp) ERROR called unimplemented method: GetAnchor1(" << std::endl;
    return Vector3f();
}

Vector3f Hinge2JointImp::GetAnchor2()
{
    //dJointID ODEJoint = (dJointID) jointID;
    //dReal anchor[3];
    //dJointGetHinge2Anchor2(ODEJoint, anchor);
    //Vector3f pos = Vector3f(anchor[0],anchor[1],anchor[2]);
    //return pos;
	std::cerr << "(Hinge2JointImp) ERROR called unimplemented method: GetAnchor2(" << std::endl;
    return Vector3f();
}

float Hinge2JointImp::GetAngle()
{
    //dJointID ODEJoint = (dJointID) jointID;
    //return gRadToDeg(dJointGetHinge2Angle1(ODEJoint));
	std::cerr << "(Hinge2JointImp) ERROR called unimplemented method: GetAngle(" << std::endl;
    return 0.0f;
}

float Hinge2JointImp::GetAngleRate1()
{
    //dJointID ODEJoint = (dJointID) jointID;
    //return gRadToDeg(dJointGetHinge2Angle1Rate(ODEJoint));
	std::cerr << "(Hinge2JointImp) ERROR called unimplemented method: GetAngleRate1(" << std::endl;
    return 0.0f;
}

float Hinge2JointImp::GetAngleRate2()
{
    //dJointID ODEJoint = (dJointID) jointID;
    //return gRadToDeg(dJointGetHinge2Angle2Rate(ODEJoint));
	std::cerr << "(Hinge2JointImp) ERROR called unimplemented method: GetAngleRate2(" << std::endl;
    return 0.0f;
}

void Hinge2JointImp::Attach(BodyInt *bodyID1, BodyInt *bodyID2){
	std::cout << "hinge2joint attach called" <<std::endl;
	btJointWrapper *wrap = (btJointWrapper *)jointID;
	btTypedConstraint *joint = wrap->joint;
	btHinge2Constraint *hinge = 0;
	//TODO: connect NULL with another body
	if(!bodyID1 || !bodyID2){
		std::cerr << "(Hinge2JointImp) ERROR can't attach a body to NULL yet" << std::endl;
		//return;
	}
	btGeom *rbA = bodyID1?static_cast<BodyImp *>(bodyID1)->btID:0;
	btGeom *rbB = bodyID2?static_cast<BodyImp *>(bodyID2)->btID:0;

	btVector3 anchor(0.0,0.0,0.0);
	btVector3 anchor2(0.0,0.0,-4.0);
	btVector3 axis1(0.0,0.0,1.0);
	btVector3 axis2(0.0,1.0,0.0);
	static bool toogle = true;
	toogle = !toogle;
	if(wrap->added && wrap->world) 
			wrap->world->removeConstraint(wrap->joint);
		if(wrap->joint){
			if(!wrap->added){
				btRigidBody * rigidA = &wrap->joint->getRigidBodyA();
				btRigidBody * rigidB = &wrap->joint->getRigidBodyB();
				delete wrap->joint;
				if(rigidA)
				{
					delete rigidA;
				}

				if(rigidB)
				{
					delete rigidB;
				}
						
			}
			else
				delete wrap->joint;
		}
	if(wrap->joint){
		//lastWorld->removeConstraint(wrap->joint);
		//delete wrap->joint;
	}
		if(rbB)
		{
			btTransform trans5 = rbB->obj->getWorldTransform();
			btTransform trans6 = rbA->obj->getWorldTransform();
			trans5.setOrigin(trans5.getOrigin()-trans6.getOrigin());
			trans6.setOrigin(btVector3(0.0,0.0,0.0));
			trans6.setBasis(trans6.getBasis().transpose());
			trans5 = trans6 * trans5;
			
			//std::cout << "x:" << trans5.getOrigin().x() << std::endl
			//	      << " y:" << trans5.getOrigin().y()<< std::endl
			//	      << " z:" << trans5.getOrigin().z()<< std::endl
			//	      << " w:" << trans5.getOrigin().w() << std::endl;

			
			btScalar yaw, pitch, roll;
			trans5.getBasis().getEulerYPR(yaw,pitch,roll);
			//std::cout << "yaw : " << yaw*180.0/M_PI << std::endl
			//		  << "pitch: " << pitch*180.0/M_PI << std::endl
			//		  << "roll: " << roll*180.0/M_PI << std::endl;
			
			trans5.setOrigin((trans5.getOrigin()*0.5));
			trans5.getBasis().setEulerYPR(yaw*0.5,pitch*0.5,roll*0.5);
			trans6.getBasis().setEulerYPR(yaw*-0.5,pitch*-0.5,roll*-0.5);
			trans6.setOrigin(trans5.getOrigin()*-1.0);
			//((btRigidBody *)rbA->obj)->updateInertiaTensor();
			//((btRigidBody *)rbB->obj)->updateInertiaTensor();
			hinge = new btHinge2Constraint (*((btRigidBody *)rbA->obj),*((btRigidBody *)rbB->obj),rbA->obj->getWorldTransform().getOrigin()+btVector3(0.01,0,0),btVector3(0,0,1),btVector3(1,0,0));//anchor,anchor2,axis1,-1.0*axis1);
			hinge->setLowerLimit(0);
			hinge->setUpperLimit(0);
			hinge->enableSpring(0,false);
			hinge->enableSpring(1,false);
			hinge->enableSpring(2,false);
			hinge->setLinearLowerLimit(btVector3(0,0,0));
			hinge->setLinearUpperLimit(btVector3(0,0,0));
			lastWorld->addConstraint(hinge,true);
			
		}
		else
		{
			return;
			//btRigidBody *rbC = new btRigidBody(0.0,new btDefaultMotionState(rbA->obj->getWorldTransform()),0);
			//rbC->activate(true);
			//rbC->setActivationState(DISABLE_DEACTIVATION);
			
			//hinge = new btHinge2Constraint (*((btRigidBody *)rbA->obj),*rbC,btTransform::getIdentity(),btTransform::getIdentity(),true);
			//lastWorld->removeRigidBody((btRigidBody *)rbA->obj);
			//lastWorld->addRigidBody((btRigidBody *)rbA->obj);
			//lastWorld->addRigidBody(rbC);
			//joint = new btHingeConstraint (*((btRigidBody *)rbA->obj),anchor,axis2);	
		}
		wrap->joint = hinge;
		if(wrap->world)
		{
			lastWorld->addConstraint(hinge,true);
			wrap->added=true;
		}

		
}
    