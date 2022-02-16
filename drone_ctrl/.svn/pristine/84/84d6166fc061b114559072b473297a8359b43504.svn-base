 /***************************************************************************
 * 
 *   Copyright (C) 2021 by DTU                             *
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

#include "umat.h"

void UMatRot3x3::rotateD2W(float v[3], float u[3])
{
  u[0] = v[0] * d2w[0][0] + v[1] * d2w[0][1] + v[2] * d2w[0][2];
  u[1] = v[0] * d2w[1][0] + v[1] * d2w[1][1] + v[2] * d2w[1][2];
  u[2] = v[0] * d2w[2][0] + v[1] * d2w[2][1] + v[2] * d2w[2][2];
}

void UMatRot3x3::rotateW2D(float v[3], float u[3])
{
  u[0] = v[0] * w2d[0][0] + v[1] * w2d[0][1] + v[2] * w2d[0][2];
  u[1] = v[0] * w2d[1][0] + v[1] * w2d[1][1] + v[2] * w2d[1][2];
  u[2] = v[0] * w2d[2][0] + v[1] * w2d[2][1] + v[2] * w2d[2][2];
}

void UMatRot3x3::set(float gamma, float beta, float alpha)
{
  float cg = cos(gamma);
  float sg = sin(gamma);
  float cb = cos(beta);
  float sb= sin(beta);
  float ca = cos(alpha);
  float sa = sin(alpha);
  d2w[0][0] = ca*cb;
  d2w[0][1] = ca*sb*sg - sa*cg;
  d2w[0][2] = ca*sb*cg + sa*sg;
  d2w[1][0] = sa*cb;
  d2w[1][1] = sa*sb*sg + ca*cg;
  d2w[1][2] = sa*sb*cg - ca*sg;
  d2w[2][0] = -sb;
  d2w[2][1] = cb*sg;
  d2w[2][2] = cb*cg;
  // and inverse (transposed)
  w2d[0][0] = d2w[0][0];
  w2d[0][1] = d2w[1][0];
  w2d[0][2] = d2w[2][0];
  w2d[1][0] = d2w[0][1];
  w2d[1][1] = d2w[1][1];
  w2d[1][2] = d2w[2][1];
  w2d[2][0] = d2w[0][2];
  w2d[2][1] = d2w[1][2];
  w2d[2][2] = d2w[2][2];
}
