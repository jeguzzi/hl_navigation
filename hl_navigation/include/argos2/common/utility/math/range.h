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

#ifndef CRANGE_H
#define CRANGE_H

#include "../datatypes/datatypes.h"
#include "../string_utilities.h"
//#include <argos2/common/utility/configuration/argos_exception.h>
#include <iostream>

namespace argos {

   template<typename T> class CRange {

   public:

      CRange(const T& t_min,
             const T& t_max) :
         m_tMin(t_min),
         m_tMax(t_max),
         m_tSpan(m_tMax - m_tMin) {
	   // ARGOS_ASSERT(t_min < t_max,
           //           "Error initializing CRange(" <<
           //           t_min << ", " << t_max << "): " <<
           //           t_min << " is not < " << t_max);
      }

      inline T GetMin() const {
         return m_tMin;
      }

      inline void SetMin(const T& t_min) {
        // ARGOS_ASSERT(t_min < m_tMax,
        //              "Error setting min CRange bound (" <<
        //              t_min << "): " <<
        //              t_min << " is not < " << m_tMax);
         m_tMin = t_min;
         /* Same as, but faster than
            m_tSpan = m_tMax - m_tMin; */
         m_tSpan = m_tMax;
         m_tSpan -= m_tMin;
      }

      inline T GetMax() const {
         return m_tMax;
      }

      inline void SetMax(const T& t_max) {
	//ARGOS_ASSERT(m_tMin < t_max,
        //              "Error setting max CRange bound (" <<
        //              t_max << "): " <<
        //              m_tMin << " is not < " << t_max);
         m_tMax = t_max;
         /* Same as, but faster than
            m_tSpan = m_tMax - m_tMin; */
         m_tSpan = m_tMax;
         m_tSpan -= m_tMin;
      }

      inline T GetSpan() const {
         return m_tSpan;
      }

      inline void Set(const T& t_min, const T& t_max) {
	//ARGOS_ASSERT(t_min < t_max,
        //              "Error setting CRange bounds (" <<
        //              t_min << ", " << t_max << "): " <<
        //              t_min << " is not < " << t_max);
         m_tMin = t_min;
         m_tMax = t_max;
         /* Same as, but faster than
            m_tSpan = m_tMax - m_tMin; */
         m_tSpan = m_tMax;
         m_tSpan -= m_tMin;
      }

      inline bool WithinMinBoundIncludedMaxBoundIncluded(const T& t_value) const {
         return t_value >= m_tMin && t_value <= m_tMax;
      }

      inline bool WithinMinBoundIncludedMaxBoundExcluded(const T& t_value) const {
         return t_value >= m_tMin && t_value < m_tMax;
      }

      inline bool WithinMinBoundExcludedMaxBoundIncluded(const T& t_value) const {
         return t_value > m_tMin && t_value <= m_tMax;
      }

      inline bool WithinMinBoundExcludedMaxBoundExcluded(const T& t_value) const {
         return t_value > m_tMin && t_value < m_tMax;
      }

      inline void TruncValue(T& t_value) const {
         if (t_value > m_tMax) t_value = m_tMax;
         if (t_value < m_tMin) t_value = m_tMin;
      }

      inline Real NormalizeValue(const T& t_value) const {
         T tTmpValue(t_value);
         TruncValue(tTmpValue);
         return static_cast<Real>(tTmpValue - m_tMin) /
                static_cast<Real>(m_tSpan);
      }

      template <typename U> void MapValueIntoRange(U& t_output_value,
                                                   const T& t_input_value,
                                                   const CRange<U>& c_range) const {
         t_output_value = (NormalizeValue(t_input_value) *
                           c_range.GetSpan()) + c_range.GetMin();
      }

      inline void WrapValue(T& t_value) const {
         /*
          * To wrap a value inside this range, we need two while loops.
          * The first one is necessary for t_value greater than the max bound;
          * the second one for t_value smaller than the min bound.
          * In the first one, we keep subtracting the span until t_value
          * falls into the range; in the second one, the span is summed.
          * It could happen that one calls this function with a t_value that is
          * too large or too small wrt the span. Due to the implementation of
          * floating-point values, a small value summed to or subtracted from
          * a much larger one could result in no change for the large value.
          * Thus, the while loop would be infinite.
          * To prevent this from happening, we first check if summing or subtracting
          * the span from t_value changes t_value's value. If so, we continue with
          * the loop. Otherwise, we bomb out and blame the caller for feeding a
          * value that cannot be wrapped.
          */
         T tBuffer;
         if(t_value > m_tMax) {
            tBuffer = t_value;
            t_value -= m_tSpan;
            if(tBuffer > t_value) {
               while(t_value > m_tMax) t_value -= m_tSpan;
            }
            else {
	      //THROW_ARGOSEXCEPTION("[BUG] The value " << t_value << " cannot be wrapped in range [" << *this << "]");
            }
         }
         if(t_value < m_tMin) {
            tBuffer = t_value;
            t_value += m_tSpan;
            if(tBuffer < t_value) {
               while(t_value < m_tMin) t_value += m_tSpan;
            }
            else {
	      //THROW_ARGOSEXCEPTION("[BUG] The value " << t_value << " cannot be wrapped in range [" << *this << "]");
            }
         }
      }

      inline friend std::ostream& operator<<(std::ostream& os,
                                             const CRange& c_range) {
         os << c_range.m_tMin << ":"
            << c_range.m_tMax;
         return os;
      }

      inline friend std::istream& operator>>(std::istream& is,
                                             CRange& c_range) {
         T tValues[2];
         ParseValues<T> (is, 2, tValues, ':');
         c_range.Set(tValues[0], tValues[1]);
         return is;
      }

   private:

      T m_tMin;
      T m_tMax;
      T m_tSpan;

   };

}

#endif
