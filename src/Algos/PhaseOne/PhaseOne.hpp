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
#ifndef __NOMAD400_PHASE_ONE__
#define __NOMAD400_PHASE_ONE__

#include "../../Eval/EvalPoint.hpp"

#include "../../Algos/Mads/Mads.hpp"

#include "../../Algos/AlgoStopReasons.hpp"

#include "../../nomad_nsbegin.hpp"

/// Class for phase one search of MADS to satisfy Extreme Barrier (EB) constraints.
/**
 This algorithm is called as a SearchMethod for Mads that modifies the problem to minimize the infeasibility of EB constraints. This is done when initial point has infeasible EB constraint. Then a sub-optimization using Mads solves the modified problem. Once completed, the parent Mads continues on the regular problem. Points for which an EB constraint is infeasible have their objective set to infinity.
 */
class PhaseOne: public Algorithm
{
private:
    
    std::shared_ptr<Mads>    _mads;
    std::shared_ptr<AlgoStopReasons<MadsStopType>>    _madsStopReasons;
    
    
    /**
      The list of ::BBOutputType parameters used for this Phase One.
      Used to recompute h values at the end of Phase One.
      Since the recompute methods are static, this member
      needs to be static.
     */
    static BBOutputTypeList _bboutputtypes;
    
public:
    /// Constructor
    /**
     \param parentStep    The parent of this step -- \b IN.
     \param stopReasons   The Phase One stop reasons -- \b IN/OUT.
     \param runParams     Parameters for algorithm -- \b IN.
     \param refPbParams   Parameters for original optimization problem. Phase One use its own copy -- \b IN.
     */
    explicit PhaseOne(const Step* parentStep,
                      std::shared_ptr<AlgoStopReasons<PhaseOneStopType>> stopReasons,
                      const std::shared_ptr<RunParameters>& runParams,
                      const std::shared_ptr<PbParameters>& refPbParams)
    : Algorithm(parentStep, stopReasons, runParams, std::make_shared<PbParameters>(*refPbParams)),
    _mads(nullptr)
    {
        init();
    }
    virtual ~PhaseOne() {}
    
    static void setBBOutputTypes(const BBOutputTypeList& bboutputtypes) { _bboutputtypes = bboutputtypes; }
    
    /**
     - Setup EvalPoint success computation to be based on h rather than f.
     - Recompute points in cache.
     - Setup stop if feasible criterion.
     - Setup Mads
     
     */
    virtual void    startImp() override;
    virtual bool    runImp()   override;
    virtual void    endImp()   override;
    
    virtual void readInformationForHotRestart() override;
    
private:
    /// Helper for constructor
    void init();
    
    /*------------------------*/
    /* Private helper methods */
    /*------------------------*/
    /**
     Static function called by Cache::processOnAllPoints().
     */
    static void recomputeH(EvalPoint& evalPoint);
    
    /**
     Static function called by Cache::processOnAllPoints().
     */
    static void recomputeHPB(EvalPoint& evalPoint);
    
};

#include "../../nomad_nsend.hpp"

#endif // __NOMAD400_PHASE_ONE__
