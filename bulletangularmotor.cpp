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
#include "bulletangularmotor.h"

using namespace oxygen;
using namespace salt;

AngularMotorImp::AngularMotorImp() : JointImp()
{

}

long AngularMotorImp::CreateAngularMotor(long worldID){
    //dWorldID ODEWorld = (dWorldID) worldID;
    //dJointID ODEJoint = dJointCreateAMotor(ODEWorld, 0);
    //return (long) ODEJoint;
	std::cerr << "(AngularMotorImp) ERROR called unimplemented method: CreateAngularMotor(" << std::endl;
	return 0;
}

void AngularMotorImp::SetModeUserMode(long jointID){
    //dJointID ODEJoint = (dJointID) jointID;
    //dJointSetAMotorMode(ODEJoint, dAMotorUser);
	std::cerr << "(AngularMotorImp) ERROR called unimplemented method: SetModeUserMode(" << std::endl;
}

void AngularMotorImp::SetModeEulerMode(long jointID){
    //dJointID ODEJoint = (dJointID) jointID;
    //dJointSetAMotorMode(ODEJoint, dAMotorEuler);
	std::cerr << "(AngularMotorImp) ERROR called unimplemented method: SetModeEulerMode(" << std::endl;
}

int AngularMotorImp::GetMode(long jointID){
    //dJointID ODEJoint = (dJointID) jointID;
    //int motorMode = dJointGetAMotorMode(ODEJoint);
    //if (motorMode = dAMotorUser) 
    //    return 0;
    ////motor is in Euler mode
    //return 1; 
	std::cerr << "(AngularMotorImp) ERROR called unimplemented method: GetMode(" << std::endl;
    return 0;
}

void AngularMotorImp::SetNumAxes(int num, long jointID){
    //dJointID ODEJoint = (dJointID) jointID;
    //dJointSetAMotorNumAxes(ODEJoint, num);
	std::cerr << "(AngularMotorImp) ERROR called unimplemented method: SetNumAxes(" << std::endl;
}

int AngularMotorImp::GetNumAxes(long jointID){
    //dJointID ODEJoint = (dJointID) jointID;
    //return dJointGetAMotorNumAxes(ODEJoint);
	std::cerr << "(AngularMotorImp) ERROR called unimplemented method: GetNumAxes(" << std::endl;
    return 1;
}

void AngularMotorImp::SetMotorAxis(int idx, int anchor, Vector3f axis, long jointID){
    //dJointID ODEJoint = (dJointID) jointID;
    //
    //dJointSetAMotorAxis (ODEJoint, idx, anchor,
    //                    axis[0], axis[1], axis[2]);
	std::cerr << "(AngularMotorImp) ERROR called unimplemented method: SetMotorAxis(" << std::endl;
}

int AngularMotorImp::GetAxisAnchor(int idx, long jointID){
    //dJointID ODEJoint = (dJointID) jointID;
    //return dJointGetAMotorAxisRel(ODEJoint, idx);
	std::cerr << "(AngularMotorImp) ERROR called unimplemented method: GetAxisAnchor(" << std::endl;
    return -1;
}

Vector3f AngularMotorImp::GetMotorAxis(int idx, long jointID){
    //dJointID ODEJoint = (dJointID) jointID;
    //dVector3 dAxis;
    //dJointGetAMotorAxis(ODEJoint,idx,dAxis);
    //return Vector3f(dAxis[0],dAxis[1],dAxis[2]);
	std::cerr << "(AngularMotorImp) ERROR called unimplemented method: GetMotorAxis(" << std::endl;
    return Vector3f();
}

void AngularMotorImp::SetAxisAngle(int idx, float degAngle, long jointID)
{
    //dJointID ODEJoint = (dJointID) jointID;
    //dJointSetAMotorAngle(ODEJoint, idx, gDegToRad(degAngle));
	std::cerr << "(AngularMotorImp) ERROR called unimplemented method: SetAxisAngle(" << std::endl;
}

float AngularMotorImp::GetAxisAngle(int idx, long jointID)
{
    //dJointID ODEJoint = (dJointID) jointID;
    //return gRadToDeg(dJointGetAMotorAngle(ODEJoint, idx));
	std::cerr << "(AngularMotorImp) ERROR called unimplemented method: GetAxisAngle(" << std::endl;
    return 0.0f;
}

float AngularMotorImp::GetAxisAngleRate(int idx, long jointID)
{
    //dJointID ODEJoint = (dJointID) jointID;
    //return gRadToDeg(dJointGetAMotorAngleRate(ODEJoint,idx));
	std::cerr << "(AngularMotorImp) ERROR called unimplemented method: GetAxisAngleRate(" << std::endl;
    return 0.0f;
}
