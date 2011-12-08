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

#include "bulletuniversaljoint.h"

using namespace oxygen;
using namespace salt;

UniversalJointImp::UniversalJointImp() : Generic6DOFJointImp()
{
	std::cerr << "(UniversalJointImp) ERROR called unimplemented constructor" << std::endl;
}

long UniversalJointImp::CreateUniversalJoint(long world)
{
    //dWorldID ODEWorld = (dWorldID) world;
    //dJointID ODEJoint = dJointCreateUniversal(ODEWorld, 0);
    //return (long) ODEJoint;
	std::cerr << "(UniversalJointImp) ERROR called unimplemented method: CreateUniversalJoint(" << std::endl;
    return 0l;
}

void UniversalJointImp::SetAnchor(const Vector3f& anchor, long jointID)
{
    //dJointID ODEJoint = (dJointID) jointID;
    //dJointSetUniversalAnchor (ODEJoint, anchor[0], anchor[1], anchor[2]);
	std::cerr << "(UniversalJointImp) ERROR called unimplemented method: SetAnchor(" << std::endl;
}

Vector3f UniversalJointImp::GetAnchor1(long jointID)
{
    //dJointID ODEJoint = (dJointID) jointID;
    //dReal anchor[3];
    //dJointGetUniversalAnchor(ODEJoint, anchor);
    //Vector3f pos = Vector3f(anchor[0],anchor[1],anchor[2]);
    //return pos;
	std::cerr << "(UniversalJointImp) ERROR called unimplemented method: GetAnchor1(" << std::endl;
    return Vector3f();
}

Vector3f UniversalJointImp::GetAnchor2(long jointID)
{
    //dJointID ODEJoint = (dJointID) jointID;
    //dReal anchor[3];
    //dJointGetUniversalAnchor2(ODEJoint, anchor);
    //Vector3f pos = Vector3f(anchor[0],anchor[1],anchor[2]);
    //return pos;
	std::cerr << "(UniversalJointImp) ERROR called unimplemented method: GetAnchor2(" << std::endl;
    return Vector3f();
}

void UniversalJointImp::SetAxis1(const Vector3f & axis, long jointID)
{
    //dJointID ODEJoint = (dJointID) jointID;
    //dJointSetUniversalAxis1(ODEJoint,axis[0],axis[1],axis[2]);
	std::cerr << "(UniversalJointImp) ERROR called unimplemented method: SetAxis1(" << std::endl;
}

void UniversalJointImp::SetAxis2(const Vector3f & axis, long jointID)
{
    //dJointID ODEJoint = (dJointID) jointID;
    //dJointSetUniversalAxis2(ODEJoint,axis[0],axis[1],axis[2]);
	std::cerr << "(UniversalJointImp) ERROR called unimplemented method: SetAxis2(" << std::endl;
}

Vector3f UniversalJointImp::GetAxis1(long jointID) const
{
    //dJointID ODEJoint = (dJointID) jointID;
    //dReal axis[3];
    //dJointGetUniversalAxis1(ODEJoint, axis);
    //Vector3f vec = Vector3f(axis[0],axis[1],axis[2]);
    //return vec;
	std::cerr << "(UniversalJointImp) ERROR called unimplemented method: GetAxis1(" << std::endl;
    return Vector3f();
}

Vector3f UniversalJointImp::GetAxis2(long jointID) const
{
    //dJointID ODEJoint = (dJointID) jointID;
    //dReal axis[3];
    //dJointGetUniversalAxis2(ODEJoint, axis);
    //Vector3f vec = Vector3f(axis[0],axis[1],axis[2]);
    //return vec;
	std::cerr << "(UniversalJointImp) ERROR called unimplemented method: GetAxis2(" << std::endl;
    return Vector3f();
}

float UniversalJointImp::GetAngle1(long jointID) const
{
    //dJointID ODEJoint = (dJointID) jointID;
    //return gRadToDeg(dJointGetUniversalAngle1(ODEJoint));
	std::cerr << "(UniversalJointImp) ERROR called unimplemented method: GetAngle1(" << std::endl;
    return 0.0f;
}

float UniversalJointImp::GetAngle2(long jointID) const
{
    //dJointID ODEJoint = (dJointID) jointID;
    //return gRadToDeg(dJointGetUniversalAngle2(ODEJoint));
	std::cerr << "(UniversalJointImp) ERROR called unimplemented method: GetAngle2(" << std::endl;
    return 0.0f;
}

float UniversalJointImp::GetAngleRate1(long jointID) const
{
    //dJointID ODEJoint = (dJointID) jointID;
    //return gRadToDeg(dJointGetUniversalAngle1Rate(ODEJoint));
	std::cerr << "(UniversalJointImp) ERROR called unimplemented method: GetAngleRate1(" << std::endl;
    return 0.0f;
}

float UniversalJointImp::GetAngleRate2(long jointID) const
{
    //dJointID ODEJoint = (dJointID) jointID;
    //return gRadToDeg(dJointGetUniversalAngle2Rate(ODEJoint));
	std::cerr << "(UniversalJointImp) ERROR called unimplemented method: GetAngleRate2(" << std::endl;
    return 0.0f;
}
