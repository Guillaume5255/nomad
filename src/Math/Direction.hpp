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
/**
 \file   Direction.hpp
 \brief  Direction: Represent geometrical vectors
 \author Viviane Rochon Montplaisir
 \date   October 2017
 \see    Direction.cpp
 */

#ifndef __NOMAD400_DIRECTION__
#define __NOMAD400_DIRECTION__

#include <numeric>
#include "../Math/ArrayOfDouble.hpp"
#include "../Math/Double.hpp"

#include "../nomad_nsbegin.hpp"

/// Type of norm
enum class NormType
{
    L1,   ///< Norm L1
    L2,   ///< Norm L2
    LINF  ///< Norm LInf
};


/// Class for the representation of a direction.
/**
 A direction is defined by its size and its coordinates from ArrayOfDouble. In addition, it provides functions to calculate different norms, dot product and cosinus of the angle between two directions.
*/
class Direction : public ArrayOfDouble
{
public:
    /*-------------*/
    /* Constructor */
    /*-------------*/
    /**
     \param n   Dimension of the direction -- \b IN.
     \param val Value for all coordinates -- \b IN.
     */
    explicit Direction(const size_t n = 0, const Double &val = Double())
      : ArrayOfDouble(n, val)
    {}

    /// Copy constructors.
    Direction(const Direction& dir)
      : ArrayOfDouble(dir)
    {}

    /// Copy constructors.
    /**
     \param pt The copied object -- \b IN.
     */
    Direction(const ArrayOfDouble& pt)
      : ArrayOfDouble(pt)
    {}

    /// Assignment operator
    /**
     \param dir The object to assign -- \b IN.
     */
    Direction& operator=(const Direction &dir);

    /// Destructor.
    virtual ~Direction() {}

    /*----------*/
    /*   Norm   */
    /*----------*/
    /// Squared L2 norm
    const Double squaredL2Norm() const;

    /** Compute norm - L1, L2 or infinite norm.
     * Default is L2.
     */
    const Double norm(NormType normType = NormType::L2) const;

    /// Infinite norm
    const Double infiniteNorm() const;

    /*-------------*/
    /* Dot product */
    /*-------------*/

    /// Dot product
    /**
     \param dir1    First \c Direction -- \b IN.
     \param dir2    Second \c Direction -- \b IN.
     \return        The dot product of dir1 and dir2.
     */
    static const Double dotProduct(const Direction& dir1,
                                   const Direction& dir2);
    /// Cosine of the angle between two vectors
    /**
     \param dir1    First \c Direction -- \b IN.
     \param dir2    Second \c Direction -- \b IN.
     \return        The cosine of the angle between the two \c Directions.
     */
    static const Double cos(const Direction& dir1, const Direction& dir2);

    /// Compute a random direction on a unit N-Sphere
    /**
     \param randomDir of the desired dimension -- \b IN/OUT
     \return \c true if the direction was computed, \c false if the process failed
     */
    static bool computeDirOnUnitSphere(Direction &randomDir);

    /// Householder transformation
    /** Householder transformation to generate _nc directions from a given direction. Also computes H[i+nc] = -H[i] (completion to 2n directions).
    \param dir given direciton -- \b IN.
    \param completeTo2n completion to 2n directions -- \b IN.
    \param H matrix for Househoulder transformation -- \b OUT.
    */
    static void householder(const NOMAD::Direction &dir,
                            bool completeTo2n,
                            NOMAD::Direction ** H);

};

/// Display of \c Direction
/**
 \param d    Object to be displayed -- \b IN.
 \param out  Reference to stream -- \b IN.
 \return     Reference to stream.
 */
std::ostream& operator<< (std::ostream& out, const Direction& d);


#include "../nomad_nsend.hpp"
#endif // __NOMAD400_DIRECTION__
