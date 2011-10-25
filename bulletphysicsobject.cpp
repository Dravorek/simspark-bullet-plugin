/* -*- mode: c++; c-basic-offset: 4; indent-tabs-mode: nil -*-

   this file is part of rcssserver3D
   Fri May 9 2003
   Copyright (C) 2011 Koblenz University
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

#include "bulletphysicsobject.h"

using namespace oxygen;
//using namespace boost;
using namespace std;

PhysicsObjectImp::PhysicsObjectImp(){
}

void PhysicsObjectImp::ConvertRotationMatrix(const salt::Matrix& rot, GenericPhysicsMatrix& matrix)
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

void PhysicsObjectImp::ConvertRotationMatrix(const GenericPhysicsMatrix* matrix, salt::Matrix& rot) const
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