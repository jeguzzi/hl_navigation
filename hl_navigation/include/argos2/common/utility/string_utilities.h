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
 * @file argos2/common/utility/string_utilities.h
 *
 * This file provides the definition of some useful string-related utilities.
 *
 * @author Carlo Pinciroli - <cpinciro@ulb.ac.be>
 */

#ifndef STRING_UTILITIES_H
#define STRING_UTILITIES_H

//#include <argos2/common/utility/configuration/argos_exception.h>
#include "./datatypes/datatypes.h"
#include <string>
#include <vector>
#include <sstream>

namespace argos {

   /****************************************/
   /****************************************/

   /**
    * Converts the given parameter to a <tt>std::string</tt>.
    * Example of use:
    * <pre>
    * UInt32 unToBeConverted(22);
    * std::string strConverted = ToString(unToBeConverted);
    * </pre>
    * Internally uses an <tt>std::ostringstream</tt>. If you want to use it with a custom
    * type, you must define its <tt>std::ostream& operator<<(std::ostream&, type)</tt>.
    * @tparam T the type of the parameter to convert.
    * @param t_value the parameter to convert.
    * @return the <tt>std::string</tt>.
    */
   template<typename T> std::string ToString(const T& t_value) {
      std::ostringstream ss;
      ss.setf(std::ios::boolalpha);
      ss << t_value;
      return ss.str();
   }

   /****************************************/
   /****************************************/

   /**
    * Converts the given <tt>std::string</tt> parameter to the wanted type.
    * Example of use:
    * <pre>
    * std::string strToBeConverted("36");
    * UInt32 unConverted = FromString(strToBeConverted);
    * </pre>
    * Internally uses an <tt>std::ostringstream</tt>. If you want to use it with a custom
    * type, you must define its <tt>std::istream& operator>>(std::istream&, type)</tt>.
    * @tparam T the type of the target parameter.
    * @param str_value the string to convert.
    * @return the converted value.
    */
   template<typename T> T FromString(const std::string& str_value) {
      T tReturnValue;
      std::istringstream ss(str_value);
      ss.setf(std::ios::boolalpha);
      ss >> tReturnValue;
      return tReturnValue;
   }

   /****************************************/
   /****************************************/

   template<typename T> void ParseValues(std::istream& str_input,
                                         UInt32 un_num_fields,
                                         T* pt_field_buffer,
                                         const char ch_delimiter = '\n') {
      std::vector<std::string> s(un_num_fields);
      UInt32 i = 0;
      while(i < un_num_fields && std::getline(str_input, s[i], ch_delimiter)) {
         i++;
      }
      if (i == un_num_fields) {
         str_input.clear(); // the istream was read completely and this is fine, so set the flag to 'good'
         for(i = 0; i < un_num_fields; i++) {
            std::istringstream iss(s[i]);
            iss >> pt_field_buffer[i];
         }
      }
      else {
	//THROW_ARGOSEXCEPTION("Parse error: expected " << un_num_fields
        //                      << " values, but " << i << " have been found in \""
        //                      << str_input << "\"");
      }
   }

   /****************************************/
   /****************************************/

   template<typename T> void ParseValues(const std::string& str_input,
                                         const UInt32 un_num_fields,
                                         T* pt_field_buffer,
                                         const char ch_delimiter = '\n') {
      std::istringstream issInput(str_input);
      ParseValues(issInput, un_num_fields, pt_field_buffer, ch_delimiter);
   }

   /****************************************/
   /****************************************/

#if 0
   extern "C" {

      /**
       * Tokenizes the given string according to the wanted delimiters (by default just a " ").
       * @param str_string the string to tokenize.
       * @param vec_tokens the vector to fill with tokens.
       * @param str_delimiters the delimiters to use.
       */
      void Tokenize(const std::string& str_string,
                    std::vector<std::string>& vec_tokens,
                    const std::string& str_delimiters = " ");

      /**
       * Converts a string to upper case.
       * @param str_string the string to convert.
       * @return the upper case string.
       */
      std::string StringToUpperCase(const std::string& str_string);

      /**
       * Converts a string to lower case.
       * @param str_string the string to convert.
       * @return the lower case string.
       */
      std::string StringToLowerCase(const std::string& str_string);

      /****************************************/
      /****************************************/

      /**
       * Replaces all occurrences of the string <tt>$ARGOSINSTALLDIR</tt> into its actual value
       * set in the environment.
       * @param str_buffer the string to search and replace.
       * @return a new string where <tt>$ARGOSINSTALLDIR</tt> is substituted by its actual value.
       */
      std::string ExpandARGoSInstallDir(const std::string& str_buffer);

      /****************************************/
      /****************************************/

      /**
       * Searches into str_buffer for occurrences of str_original and substitutes them
       * with str_new.
       * @param str_buffer the string to modify.
       * @param str_original the string to search for.
       * @param str_new the substitute string.
       */
      void Replace(std::string& str_buffer,
                   const std::string& str_original,
                   const std::string& str_new);

      /****************************************/
      /****************************************/

      /**
       * Returns <tt>true</tt> if str_pattern is matched by str_input.
       * Internally uses the C function <tt>regcomp()</tt>.
       * @param str_input the input string to analyze.
       * @param str_pattern the extended regular expression to match.
       * @return <tt>true</tt> if str_pattern is matched by str_input.
       */
      bool MatchPattern(const std::string& str_input,
                        const std::string str_pattern);

      /****************************************/
      /****************************************/

   }

#endif
}

#endif
