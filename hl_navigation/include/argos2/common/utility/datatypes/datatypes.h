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
 * @file argos2/common/utility/datatypes/datatypes.h
 *
 * This file provides the basic data type definitions of ARGoS.
 * The definitions provided here are cross-platform. For instance, a argos::SInt16
 * variable will be a 16-bit signed integer on all platforms ARGoS is ported to.
 *
 * The argos::Real type wraps the native C++ types <tt>float</tt> and <tt>double</tt>.
 * Whether argos::Real is defined as <tt>float</tt> or <tt>double</tt> depends on the
 * compilation flags set when building ARGoS. If nothing was specified, which is
 * the default, argos::Real will be defined as <tt>float</tt>. Otherwise, if the
 * macro <tt>-DARGOS_DOUBLE_PRECISION</tt> is specified, Real is defined as
 * <tt>double</tt>.
 *
 * @author Carlo Pinciroli - <cpinciro@ulb.ac.be>
 */

#ifndef DATATYPES_H
#define DATATYPES_H

#ifdef __cplusplus
#include <iostream>
#endif
/**
 * Collects all ARGoS code.
 */
#ifdef __cplusplus
namespace argos {
#endif
    
   /**
    * The basic floating point type in ARGoS.
    * Real is defined either as <tt>float</tt> or <tt>double</tt>, depending on the flags set
    * for compilation. If you don't specify anything, ARGoS defaults to using <tt>float</tt>s.
    * If you compiled with <tt>-DARGOS_DOUBLE_PRECISION</tt>, ARGoS will be compiled with
    * <tt>double</tt>s.
    */
#ifdef ARGOS_DOUBLE_PRECISION
   typedef double Real;
#else
   typedef float Real;
#endif

   /**
    * 8-bit signed integer.
    */
   typedef signed char SInt8;

#ifdef __cplusplus
   /**
    * Overloaded operator to have C++ streams handle correctly 8-bit signed integers.
    * @see SInt8
    */
   inline std::ostream& operator<<(std::ostream& c_os, const SInt8 n_value) {
       c_os << static_cast<signed int>(n_value);
       return c_os;
   }
#endif
   /**
    * 8-bit unsigned integer.
    */
   typedef unsigned char UInt8;
#ifdef __cplusplus
   /**
    * Overloaded operator to have C++ streams handle correctly 8-bit unsigned integers.
    * @see UInt8
    */
   inline std::ostream& operator<<(std::ostream& c_os, const UInt8 un_value) {
       c_os << static_cast<unsigned int>(un_value);
       return c_os;
   }
#endif
   /**
    * 16-bit signed integer.
    */
   typedef signed short SInt16;
   /**
    * 16-bit unsigned integer.
    */
   typedef unsigned short UInt16;

#ifdef __dsPIC30
   /**
    * 32-bit signed integer.
    */
   typedef signed long int SInt32;
   /**
    * 32-bit unsigned integer.
    */
   typedef unsigned long int UInt32;
#else
   /**
    * 32-bit signed integer.
    */
   typedef signed int SInt32;
   /**
    * 32-bit unsigned integer.
    */
   typedef unsigned int UInt32;
#endif
   
   /**
    * 64-bit signed integer.
    */
   typedef signed long long SInt64;
   /**
    * 64-bit unsigned integer.
    */
   typedef unsigned long long UInt64;
   
   
#ifdef __cplusplus
}
#endif

#endif
