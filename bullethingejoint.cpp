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

#include "bullethingejoint.h"
#include "bulletbody.h"
#include "bulletworld.h"

using namespace oxygen;
using namespace boost;
using namespace salt;

HingeJointImp::HingeJointImp() : Generic6DOFJointImp()
{
}

long HingeJointImp::CreateHingeJoint(WorldInt *worldID)
{
	//btDiscreteDynamicsWorld *wrld = reinterpret_cast<btDiscreteDynamicsWorld *>(worldID);
	btJointWrapper *wrp = new btJointWrapper();
	btRigidBody *rbA = new btRigidBody(0.0,0,0);
	rbA->activate(true);
	rbA->setActivationState(DISABLE_DEACTIVATION);
	rbA->setUserPointer((void *)99);
	btRigidBody *rbB = new btRigidBody(0.0,0,0);
	rbB->activate(true);
	rbB->setActivationState(DISABLE_DEACTIVATION);
	rbB->setUserPointer((void *)99);
	wrp->joint = new btHingeConstraint(*rbA,*rbB,btTransform(),btTransform(),true);
	wrp->type = JT_HINGE;
	wrp->world = static_cast<WorldImp *>(worldID)->worldID;

	jointID = wrp;
	return reinterpret_cast<long>(wrp);
	//dWorldID ODEWorld = (dWorldID) worldID;
	//dJointID ODEJoint = dJointCreateHinge(ODEWorld, 0);
	//dJointSetFeedback( ODEJoint, &mFeedback );
	//return (long) ODEJoint;
}

void HingeJointImp::SetAnchor(const Vector3f& anchor)
{
	if(jointID && jointID->joint && jointID->type ==JT_HINGE)
	{
		//static_cast<btHingeConstraint *>(jointID->joint)->setAxis(btVector3(axis.x(),axis.y(),axis.z()));
		btHingeConstraint * joint = static_cast<btHingeConstraint *>(jointID->joint);
		btTransform transA = joint->getRigidBodyA().getWorldTransform();
		btTransform transB = joint->getRigidBodyB().getWorldTransform();
		btTransform newTransA = btTransform::getIdentity();
		btTransform newTransB = btTransform::getIdentity();
		
		newTransA.setOrigin(btVector3(anchor.x(),anchor.y(),anchor.z()) - transA.getOrigin());
		newTransB.setOrigin(btVector3(anchor.x(),anchor.y(),anchor.z()) - transB.getOrigin());
		
		transA.setOrigin(btVector3(0,0,0));
		transB.setOrigin(btVector3(0,0,0));
		
		transA.setBasis(transA.getBasis().transpose());
		transB.setBasis(transB.getBasis().transpose());
		
		newTransA = transA * newTransA;
		newTransB = transB * newTransB;
		
		newTransA.setBasis(joint->getAFrame().getBasis());//btMatrix3x3::getIdentity());
		newTransB.setBasis(joint->getBFrame().getBasis());//btMatrix3x3::getIdentity());

		joint->setFrames(newTransA,newTransB);
		std::cout << "Attaching hinge: " << jointID->joint << std::endl;
		std::cout << "obj1: "<< &(joint->getRigidBodyA())<< std::endl;
		std::cout << "position: "<< "X:"<<newTransA.getOrigin().x()<< " Y:"<< newTransA.getOrigin().y()<< " Z:"<< newTransA.getOrigin().z() << std::endl;
		std::cout << "rotation: "<< "axis: X:"<<newTransA.getRotation().getAxis().x()<< " Y:"<< newTransA.getRotation().getAxis().y()<< " Z:"<< newTransA.getRotation().getAxis().z() 
			<<"angle:" << newTransA.getRotation().getAngle()<< std::endl;
		
		std::cout << "obj2: "<< &(joint->getRigidBodyB())<< std::endl;
		std::cout << "position: "<< "X:"<< newTransB.getOrigin().x()<< " Y:"<< newTransB.getOrigin().y()<< " Z:"<< newTransB.getOrigin().z() << std::endl;
		std::cout << "rotation: "<< "axis: X:"<<newTransB.getRotation().getAxis().x()<< " Y:"<< newTransB.getRotation().getAxis().y()<< " Z:"<< newTransB.getRotation().getAxis().z() 
			<<"angle:" << newTransB.getRotation().getAngle()<< std::endl;
		if(jointID->world)
		{
			jointID->world->removeConstraint(joint);
			jointID->world->addConstraint(joint,true);
		}
	}
}

Vector3f HingeJointImp::GetAnchor1()
{
	btHingeConstraint *hinge = static_cast<btHingeConstraint *>(jointID->joint);
	btVector3 offset = hinge->getRigidBodyA().getWorldTransform().getBasis() * hinge->getAFrame().getOrigin();
	offset += hinge->getRigidBodyA().getWorldTransform().getOrigin();
	return salt::Vector3f(offset.x(),offset.y(),offset.z());
}

Vector3f HingeJointImp::GetAnchor2()
{
	//dJointID ODEJoint = (dJointID) jointID;
	//dReal anchor[3];
	//dJointGetHingeAnchor2(ODEJoint, anchor);
	//Vector3f pos = Vector3f(anchor[0],anchor[1],anchor[2]);
	//
	//return pos;
	std::cerr << "(HingeJointImp) ERROR called unimplemented method: GetAnchor2(" << std::endl;
	return Vector3f();
}

void HingeJointImp::SetAxis(const Vector3f& axis)
{
	if(jointID && jointID->joint && jointID->type ==JT_HINGE)
	{
		static_cast<btHingeConstraint *>(jointID->joint)->setAxis(btVector3(axis.x(),axis.y(),axis.z()));
		std::cout << "hinge: " << jointID->joint <<" setAxis to:" << 
			"X:" << axis.x() << " Y:"<<axis.y() << " Z:" << axis.z()<< std::endl;
		if(jointID->world)
		{
			jointID->world->removeConstraint(jointID->joint);
			jointID->world->addConstraint(jointID->joint,true);
		}

	}
}

Vector3f HingeJointImp::GetAxis()
{
	btVector3 axis(1.0,0.0,0.0);
	if(jointID && jointID->joint && jointID->type ==JT_HINGE)
	{
		//static_cast<btHingeConstraint *>(jointID->joint)->getA;
	}

	return Vector3f (axis[0], axis[1], axis[2]);
}

float HingeJointImp::GetAngle() const
{
	if(jointID && jointID->joint && jointID->type ==JT_HINGE)
	{
		return gRadToDeg(static_cast<btHingeConstraint *>(jointID->joint)->getHingeAngle());
	}else
		return 0.0f;
}

float HingeJointImp::GetAngleRate() const
{
	//dJointID ODEJoint = (dJointID) jointID;
	//return gRadToDeg(dJointGetHingeAngleRate(ODEJoint));
	std::cerr << "(HingeJointImp) ERROR called unimplemented method: GetAngleRate(" << std::endl;
	return 0.0f;
}

float HingeJointImp::GetTorque() const
{
	//dJointID ODEJoint = (dJointID) jointID;
	//dJointFeedback* fb = dJointGetFeedback(ODEJoint);
	//return dLENGTH(fb->t1) + dLENGTH(fb->t2);
	std::cerr << "(HingeJointImp) ERROR called unimplemented method: GetTorque(" << std::endl;
	return 0.0f;
}

void HingeJointImp::Attach(BodyInt *bodyID1, BodyInt *bodyID2){
	std::cout << "hingjoint attach called" <<std::endl;
	btJointWrapper *wrap = (btJointWrapper *)jointID;
	btTypedConstraint *joint = wrap->joint;
	btHingeConstraint *hinge = 0;
	//TODO: connect NULL with another body
	if(!bodyID1 || !bodyID2){
		std::cerr << "(BulletJointImp) ERROR can't attach a body to NULL yet" << std::endl;
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
			wrap->joint = nullptr;
			if(rigidA)
			{
				delete rigidA;
				rigidA = nullptr;
			}

			if(rigidB)
			{
				delete rigidB;
				rigidB = nullptr;
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
		btTransform trans5 = rbB->obj->getWorldTransform();
		btTransform trans6 = rbA->obj->getWorldTransform();
		trans5.setOrigin(trans5.getOrigin()-trans6.getOrigin());
		trans6.setOrigin(btVector3(0.0,0.0,0.0));
		btVector3 axis = trans6 * btVector3(1,0,0);
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

		if(trans5.getOrigin().length2()==0)
		{
			trans5.setOrigin(btVector3(0.005,0.0,0.0));
		}
		trans5.setOrigin((trans5.getOrigin()*0.5));
		trans5.getBasis().setEulerYPR(yaw*0.5,pitch*0.5,roll*0.5);
		trans6.getBasis().setEulerYPR(yaw*-0.5,pitch*-0.5,roll*-0.5);
		trans6.setOrigin(trans5.getOrigin()*-1.0);
		hinge = new btHingeConstraint (*((btRigidBody *)rbA->obj),*((btRigidBody *)rbB->obj),trans5,trans6);//anchor,anchor2,axis1,-1.0*axis1);
		hinge->setAxis(axis);
		lastWorld->addConstraint(hinge,true);
		std::cout << "Attaching hinge: " << jointID->joint <<" setAxis to:" << 
			"X:" << axis.x() << " Y:"<<axis.y() << " Z:" << axis.z()<< std::endl;
		std::cout << "obj1: "<< (btRigidBody *)rbA->obj << std::endl;
		std::cout << "position: "<< "X:"<<trans5.getOrigin().x()<< " Y:"<< trans5.getOrigin().y()<< " Z:"<< trans5.getOrigin().z() << std::endl;
		std::cout << "rotation: "<< "axis: X:"<<trans5.getRotation().getAxis().x()<< " Y:"<< trans5.getRotation().getAxis().y()<< " Z:"<< trans5.getRotation().getAxis().z() 
			<<"angle:" << trans5.getRotation().getAngle()<< std::endl;
		
		std::cout << "obj2: "<< (btRigidBody *)rbB->obj << std::endl;
		std::cout << "position: "<< "X:"<<trans6.getOrigin().x()<< " Y:"<< trans6.getOrigin().y()<< " Z:"<< trans6.getOrigin().z() << std::endl;
		std::cout << "rotation: "<< "axis: X:"<<trans6.getRotation().getAxis().x()<< " Y:"<< trans6.getRotation().getAxis().y()<< " Z:"<< trans6.getRotation().getAxis().z() 
			<<"angle:" << trans6.getRotation().getAngle()<< std::endl;

	}
	else
	{
		btRigidBody *rbC = new btRigidBody(0.0,new btDefaultMotionState(rbA->obj->getWorldTransform()),0);
		rbC->activate(true);
		rbC->setActivationState(DISABLE_DEACTIVATION);

		hinge = new btHingeConstraint (*((btRigidBody *)rbA->obj),*rbC,btTransform::getIdentity(),btTransform::getIdentity(),true);
	}
	wrap->joint = hinge;
	if(wrap->world)
	{
		lastWorld->addConstraint(hinge,true);
		wrap->added=true;
	}


}


void HingeJointImp::SetAngularMotorVelocity(int idx, float deg)
{
	//TODO: determine correct multipliers to match ODEs parameters
	btHingeConstraint *joint = static_cast<btHingeConstraint *>(jointID->joint);
	if(deg == 0 && joint->getEnableAngularMotor())
	;else
	joint->enableAngularMotor(true,deg,joint->getMaxMotorImpulse());
}

float HingeJointImp::GetAngularMotorVelocity(int idx) const
{
	//TODO: calculate current velocity instead of giving target velocity
	return static_cast<btHingeConstraint *>(jointID->joint)->getMotorTargetVelosity();
}

void HingeJointImp::SetMaxMotorForce(int idx, float f)
{
	//TODO: determine correct multipliers to match ODEs parameters
	static_cast<btHingeConstraint *>(jointID->joint)->setMaxMotorImpulse(f/10.0);
	//:TODO: test if the motor is actually running before enabling it
	if(static_cast<btHingeConstraint *>(jointID->joint)->getMotorTargetVelosity())
		static_cast<btHingeConstraint *>(jointID->joint)->enableAngularMotor(true,
		static_cast<btHingeConstraint *>(jointID->joint)->getMotorTargetVelosity(),f*10.0);
}

float HingeJointImp::GetMaxMotorForce(int idx) const
{
	return 	static_cast<btHingeConstraint *>(jointID->joint)->getMaxMotorImpulse();
}


void HingeJointImp::SetLowStopDeg(int idx, float deg)
{
	if(jointID && jointID->joint && jointID->type ==JT_HINGE)
	{
		btHingeConstraint *joint = static_cast<btHingeConstraint *>(jointID->joint);
		joint->setLimit(deg2rad(deg),joint->getUpperLimit());
	}
}

float HingeJointImp::GetLowStopDeg(int idx) const
{
	if(jointID && jointID->joint && jointID->type ==JT_HINGE)
	{
		return rad2deg((float)(static_cast<btHingeConstraint *>(jointID->joint)->getLowerLimit()));
	}else
		return 0.0f;
}

void HingeJointImp::SetHighStopDeg(int idx, float deg)
{
	if(jointID && jointID->joint && jointID->type ==JT_HINGE)
	{
		btHingeConstraint *joint = static_cast<btHingeConstraint *>(jointID->joint);
		joint->setLimit(joint->getLowerLimit(),deg2rad(deg));
	}
}

float HingeJointImp::GetHighStopDeg(int idx) const
{
	if(jointID && jointID->joint && jointID->type ==JT_HINGE)
	{
		return rad2deg((float)(static_cast<btHingeConstraint *>(jointID->joint)->getUpperLimit()));
	}else
		return 0.0f;
}

void HingeJointImp::SetBounce(int idx, float bounce)
{
	if(jointID && jointID->joint && jointID->type ==JT_HINGE)
	{
		btHingeConstraint *joint = static_cast<btHingeConstraint *>(jointID->joint);
		joint->enableAngularMotor(true,0,joint->getMaxMotorImpulse());
	}
}

float HingeJointImp::GetBounce(int idx) const
{
	return 1.0f;
}

void HingeJointImp::SetParameter(int parameter, float value){
    
}

float HingeJointImp::GetParameter(int parameter) const{
    return 0.0f;
}

void HingeJointImp::SetCFM(int idx, float cfm)
{
	if(jointID && jointID->joint && jointID->type ==JT_HINGE)
	{
		static_cast<btHingeConstraint *>(jointID->joint)->setParam(BT_CONSTRAINT_CFM,cfm);
	}
}
float HingeJointImp::GetCFM(int idx) const
{
	if(jointID && jointID->joint && jointID->type ==JT_HINGE)
	{
		return static_cast<btHingeConstraint *>(jointID->joint)->getParam(BT_CONSTRAINT_CFM);
	}else
		return 0.0f;
}
void HingeJointImp::SetStopCFM(int idx, float cfm)
{
	if(jointID && jointID->joint && jointID->type ==JT_HINGE)
	{
		static_cast<btHingeConstraint *>(jointID->joint)->setParam(BT_CONSTRAINT_STOP_CFM,cfm);
	}
}
float HingeJointImp::GetStopCFM(int idx) const
{
	if(jointID && jointID->joint && jointID->type ==JT_HINGE)
	{
		return static_cast<btHingeConstraint *>(jointID->joint)->getParam(BT_CONSTRAINT_STOP_CFM);
	}else
		return 0.0f;
}
void HingeJointImp::SetStopERP(int idx, float erp)
{
	if(jointID && jointID->joint && jointID->type ==JT_HINGE)
	{
		static_cast<btHingeConstraint *>(jointID->joint)->setParam(BT_CONSTRAINT_STOP_ERP,erp);
	}
}

float HingeJointImp::GetStopERP(int idx) const
{
	if(jointID && jointID->joint && jointID->type ==JT_HINGE)
	{
		return static_cast<btHingeConstraint *>(jointID->joint)->getParam(BT_CONSTRAINT_STOP_ERP);
	}else
		return 0.0f;
}
    