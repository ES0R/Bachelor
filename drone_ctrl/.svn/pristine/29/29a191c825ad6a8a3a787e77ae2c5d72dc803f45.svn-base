 /***************************************************************************
 * 
 *   Copyright (C) 2020 by DTU                             *
 *   jca@elektro.dtu.dk                                                    *
 *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef UMAT_H
#define UMAT_H

#include <stdint.h>
#include "main.h"

class UMatRot3x3
{
public:
  float d2w[3][3];
  float w2d[3][3];
  /**
   * Set rotation matrix as 
   * R = Ryaw * Rpitch * Rroll
   * \param all in radians, roll on x (fwd), pitch on y (left) yaw on z (up)
   */
  void set(float roll, float pitch, float yaw);
  /**
   * rotate a vector in drone coordinates to world coordinates
   * \param v in input values (x,y,z) x is fwd, y is left, z is up
   * \param u is u = R * v
   */
  void rotateD2W(float v[3], float u[3]);
  /**
   * rotate a vector in world coordinates to drone coordinates
   * - same as above, just with transposed matrix
   * \param v in input values (x,y,z) x is fwd, y is left, z is up
   * \param u is u = R' * v
   */
  void rotateW2D(float v[3], float u[3]);
  
};


#endif
