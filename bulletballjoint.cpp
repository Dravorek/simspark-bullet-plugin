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
#include "bulletballjoint.h"

using namespace oxygen;
using namespace boost;
using namespace salt;

BallJointImp::BallJointImp() : Generic6DOFJointImp()
{
	std::cerr << "(BallJointImp) ERROR called unimplemented constructor" << std::endl;
}

long BallJointImp::CreateBallJoint(WorldInt *worldID)
{
    //dWorldID ODEWorld = (dWorldID) worldID;
    //dJointID ODEJoint = dJointCreateBall(ODEWorld, 0);
    //return (long) ODEJoint;
	std::cerr << "(BallJointImp) ERROR called unimplemented method: CreateBallJoint(" << std::endl;
    return 0;
}

void BallJointImp::SetAnchor(const Vector3f& gAnchor)
{
    //dJointID ODEJoint = (dJointID) jointID;
    //dJointSetBallAnchor (ODEJoint, gAnchor[0], gAnchor[1], gAnchor[2]);
	std::cerr << "(BallJointImp) ERROR called unimplemented method: SetAnchor(" << std::endl;
}

Vector3f BallJointImp::GetAnchor1()
{
    //dJointID ODEJoint = (dJointID) jointID;
    //dReal anchor[3];
    //dJointGetBallAnchor (ODEJoint, anchor);
    //Vector3f pos = Vector3f(anchor[0],anchor[1],anchor[2]);
    //
    //return pos;
	std::cerr << "(BallJointImp) ERROR called unimplemented method: GetAnchor1(" << std::endl;
    return Vector3f();
}

Vector3f BallJointImp::GetAnchor2()
{
    //dJointID ODEJoint = (dJointID) jointID;
    //dReal anchor[3];
    //dJointGetBallAnchor2(ODEJoint, anchor);
    //Vector3f pos = Vector3f(anchor[0],anchor[1],anchor[2]);
    //
    //return pos;
	std::cerr << "(BallJointImp) ERROR called unimplemented method: GetAnchor2(" << std::endl;
    return Vector3f();
}
