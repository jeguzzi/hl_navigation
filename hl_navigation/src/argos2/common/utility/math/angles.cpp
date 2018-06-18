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
 * @file   argos2/common/utility/math/angles.cpp
 * @author Carlo Pinciroli <cpinciro@ulb.ac.be>
 * 
 * @brief Contains definitions for angle management, such as the CRadians and CDegrees classes.
 * 
 * Contains definitions for angle management, such as the CRadians and CDegrees classes, as well as
 * trigonometric and conversion functions.
 */

#include "angles.h"

namespace argos {

   const CRadians CRadians::PI(ARGOS_PI);
   const CRadians CRadians::TWO_PI(2.0f * CRadians::PI);
   const CRadians CRadians::PI_OVER_TWO(CRadians::PI / 2.0f);
   const CRadians CRadians::PI_OVER_THREE(CRadians::PI / 3.0f);
   const CRadians CRadians::PI_OVER_FOUR(CRadians::PI / 4.0f);
   const CRadians CRadians::PI_OVER_SIX(CRadians::PI / 6.0f);
   const CRadians CRadians::ZERO;

   const CRange<CRadians> CRadians::SIGNED_RANGE(-CRadians::PI, CRadians::PI);
   const CRange<CRadians> CRadians::UNSIGNED_RANGE(CRadians(), CRadians::TWO_PI);
   const Real CRadians::RADIANS_TO_DEGREES(180.0/CRadians::PI.GetValue());
   const CRange<SInt32> CRadians::ASEBA_RANGE(-32768, 32767);

   const CRange<CDegrees> CDegrees::SIGNED_RANGE(CDegrees(-180.0), CDegrees(180.0));
   const CRange<CDegrees> CDegrees::UNSIGNED_RANGE(CDegrees(0.0), CDegrees(360.0));
   const Real CDegrees::DEGREES_TO_RADIANS(CRadians::PI.GetValue()/180.0);
   const CRange<SInt16> CDegrees::ASEBA_RANGE(-32768, 32767);

}
