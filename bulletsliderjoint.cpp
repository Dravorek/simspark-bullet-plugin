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

#include "bulletsliderjoint.h"
#include "bulletworld.h"
#include "bulletrigidbody.h"

using namespace oxygen;
using namespace boost;
using namespace salt;

SliderJointImp::SliderJointImp() : Generic6DOFJointImp()
{
	//std::cerr << "(SliderJointImp) ERROR called unimplemented constructor" << std::endl;
}

long SliderJointImp::CreateSliderJoint(WorldInt * worldID)
{
    //dWorldID ODEWorld = (dWorldID) world;
    //dJointID ODEJoint = dJointCreateSlider(ODEWorld, 0);
    //return (long) ODEJoint;
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
	wrp->joint = new btSliderConstraint(*rbA,*rbB,btTransform::getIdentity(),btTransform::getIdentity(),false);
	wrp->type = JT_SLIDER;
	wrp->world = static_cast<WorldImp *>(worldID)->worldID;

	jointID = wrp;
	return reinterpret_cast<long>(wrp);
	
	//std::cerr << "(SliderJointImp) ERROR called unimplemented method CreateSliderJoint(" << std::endl;
    return (long)wrp;
}

void SliderJointImp::SetSliderAxis(Vector3f& up)
{
	if(jointID->type != JT_SLIDER) return;

	btSliderConstraint *slider = static_cast<btSliderConstraint *>(jointID->joint);

	//the 2 bodies that need to be connected
	btRigidBody *rb1 = &jointID->joint->getRigidBodyA();
	btRigidBody *rb2 = &jointID->joint->getRigidBodyB();

	//current world positions of the 2 bodies
	btTransform bdy1_wrld_trans = rb1->getWorldTransform();
	btTransform bdy2_wrld_trans = rb2->getWorldTransform();

	//transformations for the creation of the joint
	btTransform bdy1_to_joint = btTransform::getIdentity();
	btTransform inv_rotation = btTransform::getIdentity();
	btTransform bdy2_to_joint = btTransform::getIdentity();

	//calculate the offset between body1 and body2 in object local
	//coordinates of body2
	bdy2_to_joint.setOrigin(bdy1_wrld_trans.getOrigin()-bdy2_wrld_trans.getOrigin());
	inv_rotation.setRotation(btQuaternion(bdy2_wrld_trans.getRotation().getAxis().normalized(),-1.0*bdy2_wrld_trans.getRotation().getAngle()));
	//inv_rotation.setBasis(bdy2_wrld_trans.getBasis().transpose());
	bdy2_to_joint = inv_rotation * bdy2_to_joint;
	bdy2_to_joint.setBasis(btMatrix3x3::getIdentity());
	bdy2_to_joint.setOrigin(bdy2_to_joint.getOrigin());///2.0);
	
	//calculate the axis and rotation between the default axis and the new one
	btVector3 rotation_axis = btVector3(up.x(),up.y(),up.z()).normalize();
	btVector3 x_axis = btVector3(1.0,0.0,0.0);
	btVector3 adjustment_axis = x_axis.cross(rotation_axis);
	adjustment_axis = adjustment_axis.fuzzyZero()?btVector3(0.0,0.0,1.0):adjustment_axis.normalized();
	btScalar rotation = btAcos(rotation_axis.dot(x_axis));
	
	bdy1_to_joint.setBasis(bdy1_wrld_trans.getBasis().transpose() * btMatrix3x3(btQuaternion(adjustment_axis,rotation))  );
	bdy2_to_joint.setBasis(bdy2_wrld_trans.getBasis().transpose() * btMatrix3x3(btQuaternion(adjustment_axis,rotation)) );

	slider->getFrameOffsetA() = bdy1_to_joint;
	slider->getFrameOffsetB() = bdy2_to_joint;
	//slider->getFrameOffsetA().setOrigin(bdy1_to_joint.getOrigin());
	//slider->getFrameOffsetA().setBasis(bdy1_to_joint.getBasis());
}

float SliderJointImp::GetPosition()
{
    //dJointID ODEJoint = (dJointID) jointID;
    //return dJointGetSliderPosition(ODEJoint);
	//std::cerr << "(SliderJointImp) ERROR called unimplemented method GetPosition(" << std::endl;
    return 0.0f;
}

float SliderJointImp::GetPositionRate()
{
    //dJointID ODEJoint = (dJointID) jointID;
    //return dJointGetSliderPositionRate(ODEJoint);
	//std::cerr << "(SliderJointImp) ERROR called unimplemented method GetPositionRate(" << std::endl;
    return 0.0f;
}

void SliderJointImp::Attach(BodyInt *bodyID1, BodyInt *bodyID2){
	//std::cout << "sliderjoint attach called" <<std::endl;
	btJointWrapper *wrap = (btJointWrapper *)jointID;
	btTypedConstraint *joint = wrap->joint;
	btSliderConstraint *hinge = 0;
	//TODO: connect NULL with another body
	if(!bodyID1 || !bodyID2){
		//std::cerr << "(BulletJointImp) ERROR can't attach a body to NULL yet" << std::endl;
		//return;
	}
	btGeom *rbA = bodyID1?static_cast<BodyImp *>(bodyID1)->btID:0;
	btGeom *rbB = bodyID2?static_cast<BodyImp *>(bodyID2)->btID:0;

	static bool toogle = true;
	toogle = !toogle;
	if(wrap->added && wrap->world) 
			wrap->world->removeConstraint(wrap->joint);
		if(wrap->joint){
			if(!wrap->added){
				btRigidBody * rigidA = &wrap->joint->getRigidBodyA();
				btRigidBody * rigidB = &wrap->joint->getRigidBodyB();
				delete wrap->joint;
				wrap->joint=nullptr;
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
			{
				lastWorld->removeConstraint(wrap->joint);
				delete wrap->joint;
				wrap->joint = nullptr;
			}
		}
		if(rbB)
		{
			//the 2 bodies that need to be connected
			btRigidBody *rb1 = ((btRigidBody *)rbA->obj);
			btRigidBody *rb2 = ((btRigidBody *)rbB->obj);
			
			//current world positions of the 2 bodies
			btTransform bdy1_wrld_trans = rb1->getWorldTransform();
			btTransform bdy2_wrld_trans = rb2->getWorldTransform();
			
			//transformations for the creation of the joint
			btTransform bdy1_to_joint = btTransform::getIdentity();
			btTransform inv_rotation = btTransform::getIdentity();
			btTransform bdy2_to_joint = btTransform::getIdentity();
			
			//calculate the offset between body1 and body2 in object local
			//coordinates of body2
			bdy2_to_joint.setOrigin(bdy1_wrld_trans.getOrigin()-bdy2_wrld_trans.getOrigin());
			inv_rotation.setBasis(bdy2_wrld_trans.getBasis().transpose());
			bdy2_to_joint = inv_rotation * bdy2_to_joint;
			bdy2_to_joint.setBasis(btMatrix3x3::getIdentity());
			bdy2_to_joint.setOrigin(bdy2_to_joint.getOrigin());///2.0);

			bdy1_to_joint.setBasis(bdy1_wrld_trans.getBasis().transpose()  );
			bdy2_to_joint.setBasis(bdy2_wrld_trans.getBasis().transpose()  );
			
			hinge = new btSliderConstraint(*rb1,*rb2,bdy1_to_joint,bdy2_to_joint,true);//anchor,anchor2,axis1,-1.0*axis1);
			hinge->setLowerAngLimit(0);
			hinge->setUpperAngLimit(0);//10*M_PI/180.0);
			hinge->setLowerLinLimit(-1);
			hinge->setUpperLinLimit(2);
			hinge->setDbgDrawSize(2.0);
			lastWorld->addConstraint(hinge,true);
			
		}
		else
		{

			btRigidBody *rbC = new btRigidBody(0.0,new btDefaultMotionState(rbA->obj->getWorldTransform()),0);
			rbC->activate(true);
			rbC->setActivationState(DISABLE_DEACTIVATION);
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

void SliderJointImp::SetLowStopPos(int idx, float pos)
{
	static_cast<btSliderConstraint *>(jointID->joint)->setLowerLinLimit(pos);
}

float SliderJointImp::GetLowStopPos(int idx) const
{
	return static_cast<btSliderConstraint *>(jointID->joint)->getLowerLinLimit();
}

void SliderJointImp::SetHighStopPos(int idx, float pos)
{
	static_cast<btSliderConstraint *>(jointID->joint)->setUpperLinLimit(pos);
}

float SliderJointImp::GetHighStopPos(int idx) const
{
	return static_cast<btSliderConstraint *>(jointID->joint)->getUpperLinLimit();
}

void SliderJointImp::SetLowStopDeg(int idx, float deg)
{
	static_cast<btSliderConstraint *>(jointID->joint)->setLowerAngLimit(deg);
}

float SliderJointImp::GetLowStopDeg(int idx) const
{
	return static_cast<btSliderConstraint *>(jointID->joint)->getLowerAngLimit();
}

void SliderJointImp::SetHighStopDeg(int idx, float deg)
{
	static_cast<btSliderConstraint *>(jointID->joint)->setUpperAngLimit(deg);
}

float SliderJointImp::GetHighStopDeg(int idx) const
{
	return static_cast<btSliderConstraint *>(jointID->joint)->getUpperAngLimit();
}


void SliderJointImp::recreateSliderJoint(oxygen::BodyInt *bodyID1,oxygen::BodyInt *bodyID2)
{

}
void SliderJointImp::recreateSliderJoint(btRigidBody *bodyID1,btRigidBody *bodyID2)
{

}


