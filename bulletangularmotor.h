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

#ifndef BULLETANGULARMOTOR_H
#define BULLETANGULARMOTOR_H

#include <oxygen/physicsserver/int/angularmotorint.h>
#include "bulletjoint.h"

class AngularMotorImp :  public oxygen::AngularMotorInt, public JointImp
{

/** An angular motor allows the relative angular velocities of two
    bodies to be controlled. The angular velocity can be controlled on
    up to three axes, allowing torque motors and stops to be set for
    rotation about those axes.
    
    See physicsserver/int/angularmotorint for a documentation.
*/

public:
    AngularMotorImp();
    long CreateAngularMotor(oxygen::WorldInt* worldID);
    void SetModeUserMode();
    void SetModeEulerMode();
    int GetMode();
    void SetNumAxes(int num);
    int GetNumAxes();
    void SetMotorAxis(int idx, int anchor, salt::Vector3f axis);
    int GetAxisAnchor(int idx);
    salt::Vector3f GetMotorAxis(int idx);
    void SetAxisAngle(int idx, float degAngle);
    float GetAxisAngle(int idx);
    float GetAxisAngleRate(int idx);
};

DECLARE_CLASS(AngularMotorImp);

#endif //BULLETANGULARMOTOR_H
