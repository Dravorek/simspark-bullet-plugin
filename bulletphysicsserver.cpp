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

#include "bulletphysicsserver.h"

PhysicsServerImp::PhysicsServerImp(){
	std::cerr << "(PhysicsServerImp) ERROR called unimplemented constructor" << std::endl;
}

PhysicsServerImp::~PhysicsServerImp(){
	std::cerr << "(PhysicsServerImp) ERROR called unimplemented destructor" << std::endl;
    //dCloseODE();
}

void PhysicsServerImp::InitEngine()
{
	std::cerr << "(PhysicsServerImp) ERROR called unimplemented method InitEngine(" << std::endl;
    //dInitODE();    
}
