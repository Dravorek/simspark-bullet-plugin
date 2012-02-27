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

#include "bulletjoint.h"
#include "bulletbody.h"

using namespace oxygen;
using namespace boost;
using namespace std;
using namespace salt;

extern btDiscreteDynamicsWorld *lastWorld;

JointImp::JointImp() : PhysicsObjectImp() , jointID(0), userPointer(NULL)
{
}

Joint* JointImp::GetJoint(){
    btJointWrapper *wrap = (btJointWrapper *)jointID;
	btTypedConstraint *joint = wrap->joint;
	return (Joint *)userPointer;//joint->getUserConstraintPtr();
	//dJointID JointImp = (dJointID) jointID;
    //return static_cast<Joint*>(dJointGetData(JointImp));
	//std::cerr << "(JointImp) ERROR called unimplemented method GetJoint(" << std::endl;
	//return 0;
}

bool JointImp::AreConnected(BodyInt *bodyID1, BodyInt *bodyID2){
    //dBodyID ODEBody1 = (dBodyID) bodyID1;
    //dBodyID ODEBody2 = (dBodyID) bodyID2;
    //return dAreConnected(ODEBody1, ODEBody2) == 1;
	std::cerr << "(JointImp) WARNING called unfinished (inefficient) method AreConnected(" << std::endl;
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

bool JointImp::AreConnectedExcluding(BodyInt *bodyID1, BodyInt *bodyID2, int joint_type){
    //dBodyID ODEBody1 = (dBodyID) bodyID1;
    //dBodyID ODEBody2 = (dBodyID) bodyID2;
    //return dAreConnectedExcluding(ODEBody1, ODEBody2, joint_type) == 1;
	std::cerr << "(JointImp) ERROR called unfinished method AreConnectedExcluding(" << std::endl;

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

void JointImp::OnLink( Joint* joint)
{
	btJointWrapper *wrap = (btJointWrapper *)jointID;
	btTypedConstraint *bulletjoint = wrap->joint;
	bulletjoint->setUserConstraintPtr(this);
	userPointer = joint;

    //dJointID JointImp = (dJointID) jointID;
    //dJointSetData(JointImp, joint);
	//std::cerr << "(JointImp) ERROR called unimplemented method OnLink(" << std::endl;
}

void JointImp::Attach(BodyInt *bodyID1, BodyInt *bodyID2)
{
	btJointWrapper *wrap = (btJointWrapper *)jointID;
	btTypedConstraint *joint = wrap->joint;
	
	//TODO: connect NULL with another body
	if(!bodyID1 || !bodyID2){
		std::cerr << "(BulletJointImp) ERROR can't attach a body to NULL yet" << std::endl;
		return;
	}
	btGeom *body1 = static_cast<BodyImp *>(bodyID1)->btID;
	btGeom *body2 = static_cast<BodyImp *>(bodyID2)->btID;
	if(body1 && body2 && (!body1->isRigidBody || !body2->isRigidBody) )
	{
		//TODO: doesn't catch edge cases like connecting NULL to a collider
		std::cerr << "(BulletJointImp) ERROR can't attach a collider to a joint" << std::endl;
		return;
	}
	
	//btRigidBody *bodyA = (btRigidBody *)body1->obj;
	//btRigidBody *bodyB = (btRigidBody *)body2->obj;

	btGeom * rbA = body1;//(btGeom *)bodyID1;
	btGeom * rbB = body2;//(btGeom *)bodyID2;
	//btUniversalConstraint * joint;
	btVector3 anchor(0.0,0.0,0.0);
	btVector3 axis1(0.0,1.0,0.0);
	btVector3 axis2(1.0,0.0,0.0);

		
	switch(wrap->type)
	{
	case JT_FIXED:
		if(wrap->added && wrap->world) wrap->world->removeConstraint(wrap->joint);
		if(wrap->joint)
		{
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
		joint = new btUniversalConstraint(*((btRigidBody *)rbA->obj),*((btRigidBody *)rbB->obj),anchor,axis1,axis2);
		reinterpret_cast<btUniversalConstraint *>(joint)->setLinearLowerLimit(btVector3(0.0,0.0,0.0));
		reinterpret_cast<btUniversalConstraint *>(joint)->setLinearUpperLimit(btVector3(0.0,0.0,0.0));
		reinterpret_cast<btUniversalConstraint *>(joint)->setAngularLowerLimit(btVector3(0.0,0.0,0.0));
		reinterpret_cast<btUniversalConstraint *>(joint)->setAngularUpperLimit(btVector3(0.0,0.0,0.0));
		wrap->joint = joint;
		if(wrap->world)
		{
			wrap->world->addConstraint(wrap->joint,true);
			wrap->added=true;
		}
		break;
	case JT_HINGE:
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

		if(rbB)
			joint = new btHingeConstraint (*((btRigidBody *)rbA->obj),*((btRigidBody *)rbB->obj),anchor,anchor,axis1,axis2);
		else
			joint = new btHingeConstraint (*((btRigidBody *)rbA->obj),anchor,axis2);	
		wrap->joint = joint;
		if(wrap->world)
		{
			wrap->world->addConstraint(wrap->joint,true);
			wrap->added=true;
		}
		break;
	}

	//:TODO: getRigidBodyA() returns a mutable reference, maybe use that
	//:HACK: just doing type discrimination based on polymorphism is smarter
	//btTypedConstraintType type = joint->getConstraintType();
	//btTypedConstraint *newJoint;
	//switch(type){
	//case POINT2POINT_CONSTRAINT_TYPE:
	//	//newJoint = new btPoint2PointConstraint(*bodyA,*bodyB,btVector3(),btVector3());
	//	break;
	//case HINGE_CONSTRAINT_TYPE:
	//	//newJoint = new btPoint2PointConstraint(*bodyA,*bodyB,btVector3(),btVector3());
	//	break;
	//case CONETWIST_CONSTRAINT_TYPE:
	//	break;
	//case D6_CONSTRAINT_TYPE:
	//	break;
	//case CONTACT_CONSTRAINT_TYPE:
	//	break;
	//}
	//dJointAttach(JointImp, ODEBody1, ODEBody2);
	std::cerr << "(JointImp) ERROR called unimplemented method Attach(" << std::endl;
}

int JointImp::GetType() const
{
    //dJointID JointImp = (dJointID) jointID;
    //return dJointGetType(JointImp);
	//std::cerr << "(JointImp) ERROR called unimplemented method GetType(" << std::endl;
	btJointWrapper *wrap = (btJointWrapper *)jointID;
	btTypedConstraint *joint = wrap->joint;
	
	return joint->getConstraintType();
}

BodyInt *JointImp::GetBodyID(int idx)
{
    //dJointID JointImp = (dJointID) jointID;
    //dBodyID ODEBodyID = dJointGetBody(JointImp, idx);
    //return (long) ODEBodyID;
	//std::cerr << "(JointImp) ERROR called unimplemented method GetBodyID(" << std::endl;
    btJointWrapper *wrap = (btJointWrapper *)jointID;
	btTypedConstraint *joint = wrap->joint;
	
	if(idx==0)
	{
		return static_cast<BodyInt *>(joint->getRigidBodyA().getUserPointer());
	}
	else if(idx==1)
	{
		return static_cast<BodyInt *>(joint->getRigidBodyB().getUserPointer());
	}
	else
		return 0l;
}

void JointImp::EnableFeedback(bool enable,
                              boost::shared_ptr<GenericJointFeedback>& feedback)
{
    //dJointID JointImp = (dJointID) jointID;

    //if (enable)
    //    {
    //        if (feedback.get() == 0)
    //            {
    //                feedback = boost::shared_ptr<GenericJointFeedback>(
    //                    (GenericJointFeedback*) new dJointFeedback());
    //                memset(feedback.get(),0,sizeof(dJointFeedback));
    //            }
    //    } else
    //        {
    //            if (feedback.get() != 0)
    //                {
    //                    feedback.reset();
    //                }
    //        }

    //dJointFeedback* ODEFeedback = (dJointFeedback*) feedback.get();

    //dJointSetFeedback(JointImp,ODEFeedback);
	std::cerr << "(JointImp) ERROR called unimplemented method EnableFeedback(" << std::endl;
}

bool JointImp::FeedbackEnabled() const
{
    btJointWrapper *wrap = (btJointWrapper *)jointID;
	btTypedConstraint *joint = wrap->joint;
	
	return joint->needsFeedback();
}

Vector3f JointImp::GetFeedbackForce(int idx,
                                    boost::shared_ptr<GenericJointFeedback> feedback) const
{
	//:TODO: not actually called anywhere currently. delete or change because there is no equivalent in Bullet
    //dJointFeedback* fb = (dJointFeedback*) feedback.get();
    //if (fb == 0)
    //    {
    //        return Vector3f(0,0,0);
    //    }

    //switch (idx)
    //    {
    //    case 0 :
    //        return Vector3f(
    //                        fb->f1[0],
    //                        fb->f1[1],
    //                        fb->f1[2]
    //                        );

    //    case 1 :
    //        return Vector3f(
    //                        fb->f2[0],
    //                        fb->f2[1],
    //                        fb->f2[2]
    //                        );

    //    default:
    //        return Vector3f(0,0,0);
    //    }
	std::cerr << "(JointImp) ERROR called unimplemented method GetFeedbackForce(" << std::endl;
    return Vector3f();
}

Vector3f JointImp::GetFeedbackTorque(int idx,
                                     boost::shared_ptr<GenericJointFeedback> feedback) const
{
	//:TODO: not actually called anywhere currently. delete or change because there is no equivalent in Bullet
    //dJointFeedback* fb = (dJointFeedback*) feedback.get();
    //if (fb == 0)
    //    {
    //        return Vector3f(0,0,0);
    //    }

    //switch (idx)
    //    {
    //    case 0 :
    //        return Vector3f(
    //                        fb->t1[0],
    //                        fb->t1[1],
    //                        fb->t1[2]
    //                        );

    //    case 1 :
    //        return Vector3f(
    //                        fb->t2[0],
    //                        fb->t2[1],
    //                        fb->t2[2]
    //                        );

    //    default:
    //        return Vector3f(0,0,0);
    //    }
	std::cerr << "(JointImp) ERROR called unimplemented method GetFeedbackTorque(" << std::endl;
    return Vector3f();
}

void JointImp::SetFudgeFactor(int idx, float fudge_factor)
{
    //SetParameter(dParamFudgeFactor + (idx * dParamGroup), fudge_factor, jointID);
	std::cerr << "(JointImp) ERROR called unimplemented method SetFudgeFactor(" << std::endl;
}

float JointImp::GetFudgeFactor(int idx) const
{
    //return GetParameter(dParamFudgeFactor + (idx * dParamGroup), jointID);
	std::cerr << "(JointImp) ERROR called unimplemented method GetFudgeFactor(" << std::endl;
    return 0.0f;
}

void JointImp::SetBounce(int idx, float bounce)
{
    //SetParameter(dParamBounce + (idx * dParamGroup),bounce, jointID);
	std::cerr << "(JointImp) ERROR called unimplemented method SetBounce(" << std::endl;
}

float JointImp::GetBounce(int idx) const
{
    //return GetParameter(dParamBounce + (idx * dParamGroup), jointID);
	std::cerr << "(JointImp) ERROR called unimplemented method GetBounce(" << std::endl;
    return 0.0f;
}

void JointImp::SetLowStopPos(int idx, float pos)
{
    //SetParameter(dParamLoStop + (idx * dParamGroup), pos, jointID);
	std::cerr << "(JointImp) ERROR called unimplemented method SetLowStopPos(" << std::endl;
}

float JointImp::GetLowStopPos(int idx) const
{
    //return GetParameter(dParamLoStop + (idx * dParamGroup), jointID);
	std::cerr << "(JointImp) ERROR called unimplemented method GetLowStopPos(" << std::endl;
    return 0.0f;
}

void JointImp::SetHighStopPos(int idx, float pos)
{
    //SetParameter(dParamHiStop + (idx * dParamGroup), pos, jointID);
	std::cerr << "(JointImp) ERROR called unimplemented method SetHighStopPos(" << std::endl;
}

float JointImp::GetHighStopPos(int idx) const
{
    //return GetParameter(dParamHiStop + (idx * dParamGroup), jointID);
	std::cerr << "(JointImp) ERROR called unimplemented method GetHighStopPos(" << std::endl;
    return 0.0f;
}

void JointImp::SetLowStopDeg(int idx, float deg)
{
    //SetParameter(dParamLoStop + (idx * dParamGroup), gDegToRad(deg), jointID);
	std::cerr << "(JointImp) ERROR called unimplemented method SetLowStopDeg(" << std::endl;
}

float JointImp::GetLowStopDeg(int idx) const
{
    //return gRadToDeg(GetParameter(dParamLoStop + (idx * dParamGroup), jointID));
	std::cerr << "(JointImp) ERROR called unimplemented method GetLowStopDeg(" << std::endl;
    return 0.0f;
}

void JointImp::SetHighStopDeg(int idx, float deg)
{
    //SetParameter(dParamHiStop + (idx * dParamGroup), gDegToRad(deg), jointID);
	std::cerr << "(JointImp) ERROR called unimplemented method SetHighStopDeg(" << std::endl;
}

float JointImp::GetHighStopDeg(int idx) const
{
    //return gRadToDeg(GetParameter(dParamHiStop + (idx * dParamGroup), jointID));
	std::cerr << "(JointImp) ERROR called unimplemented method GetHighStopDeg(" << std::endl;
    return 0.0f;
}

void JointImp::SetCFM(int idx, float cfm)
{
    //SetParameter(dParamCFM + (idx * dParamGroup), cfm, jointID);
	std::cerr << "(JointImp) ERROR called unimplemented method SetCFM(" << std::endl;
}

float JointImp::GetCFM(int idx) const
{
    //return GetParameter(dParamCFM + (idx * dParamGroup), jointID);
	std::cerr << "(JointImp) ERROR called unimplemented method GetCFM(" << std::endl;
    return 0.0f;
}

void JointImp::SetStopCFM(int idx, float cfm)
{
    //SetParameter(dParamStopCFM + (idx * dParamGroup), cfm, jointID);
	std::cerr << "(JointImp) ERROR called unimplemented method SetStopCFM(" << std::endl;
}

float JointImp::GetStopCFM(int idx) const
{
    //return GetParameter(dParamStopCFM + (idx * dParamGroup), jointID);
	std::cerr << "(JointImp) ERROR called unimplemented method GetStopCFM(" << std::endl;
    return 0.0f;
}

void JointImp::SetStopERP(int idx, float erp)
{
    //SetParameter(dParamStopERP + (idx * dParamGroup), erp, jointID);
	std::cerr << "(JointImp) ERROR called unimplemented method SetStopERP(" << std::endl;
}

float JointImp::GetStopERP(int idx) const
{
    //return GetParameter(dParamStopERP + (idx * dParamGroup), jointID);
	std::cerr << "(JointImp) ERROR called unimplemented method GetStopERP(" << std::endl;
    return 0.0f;
}

void JointImp::SetSuspensionERP(int idx, float erp)
{
    //SetParameter(dParamSuspensionERP + (idx * dParamGroup), erp, jointID);
	std::cerr << "(JointImp) ERROR called unimplemented method SetSuspensionERP(" << std::endl;
}

float JointImp::GetSuspensionERP(int idx) const
{
    //return GetParameter(dParamSuspensionERP + (idx * dParamGroup), jointID);
	std::cerr << "(JointImp) ERROR called unimplemented method GetSuspensionERP(" << std::endl;
    return 0.0f;
}

void JointImp::SetSuspensionCFM(int idx, float cfm)
{
    //SetParameter(dParamSuspensionCFM + (idx * dParamGroup), cfm, jointID);
	std::cerr << "(JointImp) ERROR called unimplemented method SetSuspensionCFM(" << std::endl;
}

float JointImp::GetSuspensionCFM(int idx) const
{
    //return GetParameter(dParamSuspensionCFM + (idx * dParamGroup), jointID);
	std::cerr << "(JointImp) ERROR called unimplemented method GetSuspensionCFM(" << std::endl;
    return 0.0f;
}

void JointImp::SetLinearMotorVelocity(int idx, float vel)
{
    //SetParameter(dParamVel + (idx * dParamGroup), vel, jointID);
	std::cerr << "(JointImp) ERROR called unimplemented method SetLinearMotorVelocity(" << std::endl;
}

float JointImp::GetLinearMotorVelocity(int idx) const
{
    //return GetParameter(dParamVel + (idx * dParamGroup), jointID);
	std::cerr << "(JointImp) ERROR called unimplemented method GetLinearMotorVelocity(" << std::endl;
    return 0.0f;
}

void JointImp::SetAngularMotorVelocity(int idx, float deg)
{
    //SetParameter(dParamVel + (idx * dParamGroup), gDegToRad(deg), jointID);
	std::cerr << "(JointImp) ERROR called unimplemented method SetAngularMotorVelocity(" << std::endl;
}

float JointImp::GetAngularMotorVelocity(int idx) const
{
    //return gRadToDeg(GetParameter(dParamVel + (idx * dParamGroup), jointID));
	std::cerr << "(JointImp) ERROR called unimplemented method GetAngularMotorVelocity(" << std::endl;
    return 0.0f;
}

void JointImp::SetMaxMotorForce(int idx, float f)
{
	std::cerr << "(JointImp) ERROR called unimplemented method SetMaxMotorForce(" << std::endl;
    //SetParameter(dParamFMax + (idx * dParamGroup), f, jointID);
}

float JointImp::GetMaxMotorForce(int idx) const
{
    //return GetParameter(dParamFMax + (idx * dParamGroup), jointID);
	std::cerr << "(JointImp) ERROR called unimplemented method GetMaxMotorForce(" << std::endl;
    return 0.0f;
}

void JointImp::DestroyJoint(
                            boost::shared_ptr<GenericJointFeedback> feedback)
{
	btJointWrapper *wrap = (btJointWrapper *)jointID;
	btTypedConstraint *joint = wrap->joint;
	if(wrap->added && wrap->world)
		wrap->world->removeConstraint(joint);
	delete joint;
	delete wrap;
	
    //dJointID JointImp = (dJointID) jointID;
    //EnableFeedback(false, jointID, feedback);
    //dJointDestroy(JointImp);

	std::cerr << "(JointImp) ERROR called unimplemented method DestroyJoint(" << std::endl;
}

void JointImp::SetParameter(int parameter, float value){
    //dJointID JointImp = (dJointID) jointID;
    //int jointType = dJointGetType(JointImp);
    //switch (jointType){
    //    case dJointTypeHinge: dJointSetHingeParam(JointImp, parameter, value);
    //                          break;
    //    case dJointTypeHinge2: dJointSetHinge2Param(JointImp, parameter, value);
    //                           break;
    //    case dJointTypeSlider: dJointSetSliderParam(JointImp, parameter, value);
    //                           break;
    //    case dJointTypeUniversal: dJointSetUniversalParam(JointImp, parameter, value);
    //                              break;
    //    case dJointTypeAMotor: dJointSetAMotorParam(JointImp, parameter, value);
    //                           break;
    //    default: return;
    //}
	std::cerr << "(JointImp) ERROR called unimplemented method SetParameter(" << std::endl;
}

float JointImp::GetParameter(int parameter) const{
    //dJointID JointImp = (dJointID) jointID;
    //int jointType = dJointGetType(JointImp);
    //switch (jointType){
    //    case dJointTypeHinge: return dJointGetHingeParam(JointImp, parameter);
    //    case dJointTypeHinge2: return dJointGetHinge2Param(JointImp, parameter);
    //    case dJointTypeSlider: return dJointGetSliderParam(JointImp, parameter);
    //    case dJointTypeUniversal: return dJointGetUniversalParam(JointImp, parameter);
    //    case dJointTypeAMotor: return dJointGetAMotorParam(JointImp, parameter);
    //    default: return 0;
    //}
	std::cerr << "(JointImp) ERROR called unimplemented method GetParameter(" << std::endl;
    return 0.0f;
}

float JointImp::deg2rad(float degval) const
{
	return degval*M_PI / 180.0;
}

float JointImp::rad2deg(float radval) const
{
	return radval *180.0 / M_PI;
}