/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/**
 * @file common/utility/math/vector3.h
 *
 * @brief This file provides the definition of
 *
 * @author Carlo Pinciroli - <cpinciro@ulb.ac.be>
 */

#include "vector3.h"
#include "quaternion.h"

namespace argos {

   /****************************************/
   /****************************************/

   const CVector3 CVector3::X(1.0, 0.0, 0.0);
   const CVector3 CVector3::Y(0.0, 1.0, 0.0);
   const CVector3 CVector3::Z(0.0, 0.0, 1.0);
   const CVector3 CVector3::ZERO;

   /****************************************/
   /****************************************/

   CVector3& CVector3::Rotate(const CQuaternion& c_quaternion) {
      CQuaternion cResult;
      cResult = c_quaternion;
      cResult *= CQuaternion(0.0f, m_fX, m_fY, m_fZ);
      cResult *= c_quaternion.Inverse();
      m_fX = cResult.GetX();
      m_fY = cResult.GetY();
      m_fZ = cResult.GetZ();
      return *this;
   }

   /****************************************/
   /****************************************/

}
