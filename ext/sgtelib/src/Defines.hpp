/*---------------------------------------------------------------------------------*/
/*  NOMAD - Nonlinear Optimization by Mesh Adaptive Direct Search -                */
/*                                                                                 */
/*  NOMAD - Version 4.0.0 has been created by                                      */
/*                 Viviane Rochon Montplaisir  - Polytechnique Montreal            */
/*                 Christophe Tribes           - Polytechnique Montreal            */
/*                                                                                 */
/*  The copyright of NOMAD - version 4.0.0 is owned by                             */
/*                 Sebastien Le Digabel        - Polytechnique Montreal            */
/*                 Viviane Rochon Montplaisir  - Polytechnique Montreal            */
/*                 Christophe Tribes           - Polytechnique Montreal            */
/*                                                                                 */
/*  NOMAD v4 has been funded by Rio Tinto, Hydro-Québec, NSERC (Natural Science    */
/*  and Engineering Research Council of Canada), INOVEE (Innovation en Energie     */
/*  Electrique and IVADO (The Institute for Data Valorization)                     */
/*                                                                                 */
/*  NOMAD v3 was created and developed by Charles Audet, Sebastien Le Digabel,     */
/*  Christophe Tribes and Viviane Rochon Montplaisir and was funded by AFOSR       */
/*  and Exxon Mobil.                                                               */
/*                                                                                 */
/*  NOMAD v1 and v2 were created and developed by Mark Abramson, Charles Audet,    */
/*  Gilles Couture, and John E. Dennis Jr., and were funded by AFOSR and           */
/*  Exxon Mobil.                                                                   */
/*                                                                                 */
/*  Contact information:                                                           */
/*    Polytechnique Montreal - GERAD                                               */
/*    C.P. 6079, Succ. Centre-ville, Montreal (Quebec) H3C 3A7 Canada              */
/*    e-mail: nomad@gerad.ca                                                       */
/*    phone : 1-514-340-6053 #6928                                                 */
/*    fax   : 1-514-340-5665                                                       */
/*                                                                                 */
/*  This program is free software: you can redistribute it and/or modify it        */
/*  under the terms of the GNU Lesser General Public License as published by       */
/*  the Free Software Foundation, either version 3 of the License, or (at your     */
/*  option) any later version.                                                     */
/*                                                                                 */
/*  This program is distributed in the hope that it will be useful, but WITHOUT    */
/*  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or          */
/*  FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License    */
/*  for more details.                                                              */
/*                                                                                 */
/*  You should have received a copy of the GNU Lesser General Public License       */
/*  along with this program. If not, see <http://www.gnu.org/licenses/>.           */
/*                                                                                 */
/*  You can find information on the NOMAD software at www.gerad.ca/nomad           */
/*---------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------*/
/*  sgtelib - A surrogate model library for derivative-free optimization               */
/*  Version 2.0.2                                                                      */
/*                                                                                     */
/*  Copyright (C) 2012-2017  Sebastien Le Digabel - Ecole Polytechnique, Montreal      */ 
/*                           Bastien Talgorn - McGill University, Montreal             */
/*                                                                                     */
/*  Author: Bastien Talgorn                                                            */
/*  email: bastientalgorn@fastmail.com                                                 */
/*                                                                                     */
/*  This program is free software: you can redistribute it and/or modify it under the  */
/*  terms of the GNU Lesser General Public License as published by the Free Software   */
/*  Foundation, either version 3 of the License, or (at your option) any later         */
/*  version.                                                                           */
/*                                                                                     */
/*  This program is distributed in the hope that it will be useful, but WITHOUT ANY    */
/*  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A    */
/*  PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.   */
/*                                                                                     */
/*  You should have received a copy of the GNU Lesser General Public License along     */
/*  with this program. If not, see <http://www.gnu.org/licenses/>.                     */
/*                                                                                     */
/*  You can find information on sgtelib at https://github.com/bastientalgorn/sgtelib   */
/*-------------------------------------------------------------------------------------*/

#ifndef __SGTELIB_DEFINES__
#define __SGTELIB_DEFINES__

#include <iostream>
#include <iomanip>
#include <cmath>
#include <sstream>
#include <string.h>

#include <limits>
#include <limits.h>

// CASE Visual Studio C++ compiler
#ifdef _MSC_VER
#pragma warning(disable:4251)
#ifdef DLL_EXPORTS
#define DLL_API __declspec(dllexport) 
#else
#define DLL_API __declspec(dllimport) 
#endif
#else
#define DLL_API
#endif

// debug flag:
//#define SGTELIB_DEBUG
//#define ENSEMBLE_DEBUG

namespace SGTELIB {
  
  const double EPSILON = 1E-13;
  const double PI = 3.141592654;
  const double INF = std::numeric_limits<double>::max(); ///< Infinity
  const double NaN = std::numeric_limits<double>::quiet_NaN();

  const bool APPROX_CDF = true;
  // If true, then the lower bound of standard deviation is EPSILON. 
  // This allows to avoid flat EI and P functions. 


  enum norm_t {
    NORM_0 ,
    NORM_1 ,
    NORM_2 ,
    NORM_INF
  };



  enum scaling_t {
    SCALING_NONE ,
    SCALING_MEANSTD ,
    SCALING_BOUNDS
  };

  const scaling_t scaling_method = SCALING_MEANSTD;
  const int boolean_rounding = 2;
  // 0: no boolean scaling
  // 1: threshold = (Z_UB+Z_LB)/2;
  // 2: threshold = mean(Z)
  //const scaling_t scaling_method = SCALING_NONE;


  // model output type:
  enum model_output_t {
    NORMAL_OUTPUT ,
    FIXED_OUTPUT  ,
    BINARY_OUTPUT
  };

  // model output type:
  enum bbo_t {
    BBO_OBJ , // Objective
    BBO_CON , // Constraint
    BBO_DUM   // Dummy
  };

  enum param_domain_t {
    PARAM_DOMAIN_CONTINUOUS,
    PARAM_DOMAIN_INTEGER,
    PARAM_DOMAIN_BOOL,
    PARAM_DOMAIN_CAT,
    PARAM_DOMAIN_MISC
  };

  // FIXED => Parameter is set once and for all
  // OPTIM => Parameter is optimized so as to minimize a metric
  // MODEL_DEFINED => The surrogate model is able to define/chose the parameter
  enum param_status_t {
    STATUS_FIXED,
    STATUS_OPTIM,
    STATUS_MODEL_DEFINED
  };
    


}

#endif
