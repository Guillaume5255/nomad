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
/**
 \file   Projection.hpp
 \brief  Class for creating new points around already existing points
 \author Viviane Rochon Montplaisir
 \date   November 2019
 \see    Projection.cpp
// This class is based on get_best_projection() in the Sgtelib Model Search of NOMAD 3.
// Warning:
// This class is incomplete. Work in progress. Currently not used.
 */
/*-------------------------------------------------------------------------------------*/

#include "../Algos/CacheInterface.hpp"
#include "../Algos/Projection.hpp"
#include "../Math/RNG.hpp"

NOMAD::Projection::Projection(const NOMAD::Step* parentStep,
                              const NOMAD::EvalPointSet &oraclePoints)
  : Step(parentStep),
    IterationUtils(parentStep),
    _oraclePoints(oraclePoints),
    _displayLevel(NOMAD::OutputLevel::LEVEL_INFO),
    _cacheSgte(0),
    _mesh(nullptr),
    _frameCenter(nullptr),
    _indexSet()
{
    init();
}


NOMAD::Projection::~Projection()
{
}


void NOMAD::Projection::init()
{
    _name = "Projection";
    verifyParentNotNull();

    // Find cache points with Sgte evaluation
    NOMAD::CacheInterface cacheInterface(this);
    cacheInterface.find(NOMAD::EvalPoint::hasSgteEval, _cacheSgte);

    auto iter = dynamic_cast<const NOMAD::Iteration*>(getParentOfType<NOMAD::Iteration*>());

    if (nullptr != iter)
    {
        _mesh = iter->getMesh();
        _frameCenter = iter->getFrameCenter();
        if (_frameCenter)
        {
            buildIndexSet(_frameCenter->size());
            //_nbProjTrial = 100 * _frameCenter->size();
            _nbProjTrial = _frameCenter->size();
        }
    }


}


// Set of indexes for neighbour creation
void NOMAD::Projection::buildIndexSet(const size_t n)
{
    const size_t nbNeighbours = static_cast<size_t>(pow(2.0, n));
    // Build the set of indexes
    if (_nbProjTrial < nbNeighbours * 1.3)
    {
        // Select randomly _nbProjTrial indexes
        for (size_t i = 0; i < _nbProjTrial; i++)
        {
            _indexSet.insert(NOMAD::RNG::rand() % nbNeighbours);
        }
    }
    else
    {
        // Use all indexes
        for (size_t i = 0 ; i < nbNeighbours; i++)
        {
            _indexSet.insert(i);
        }
    }

}


void NOMAD::Projection::startImp()
{
    // Create EvalPoints and send them to EvaluatorControl
    generateTrialPoints();
}


bool NOMAD::Projection::runImp()
{
    bool projectionOk = true;

    // Return value: found better - unused.
    // TODO Ensure OPPORTUNISM is off here
    evalTrialPoints(this);

    // TODO - postprocessing?


    return projectionOk;
}


void NOMAD::Projection::endImp()
{
}


void NOMAD::Projection::projectPoint(const NOMAD::EvalPoint& oraclePoint)
{
    std::string s;

    // Non projected point
    nonProjectedPoint(oraclePoint);

    // STD projected point
    stdProjectedPoint(oraclePoint);
    // TODO compute this otherwise, if needed
    //auto f = bestEvalPoint.getF(evalType);
    //auto h = bestEvalPoint.getH(evalType);

    // Try pertubation
    std::string subStepName = "Projection candidate";
    AddOutputInfo(subStepName, true, false);

    NOMAD::EvalPointSet trySet;
    size_t nbCachePoints;

    for (auto index : _indexSet)
    {
        // Compute perturbation
        auto perturbation = computePerturbation(oraclePoint, index);

        // Loop on points of the cache
        NOMAD::CacheInterface cacheInterface(this);
        std::vector<NOMAD::EvalPoint> evalPointList;
        nbCachePoints = cacheInterface.getAllPoints(evalPointList);

        for (auto xRef : evalPointList)
        {
            auto xTry = buildProjectionTrialPoint(xRef, perturbation);
            trySet.insert(xTry);
        }
    }

    s = std::to_string(_indexSet.size()) + " perturbation vectors";
    NOMAD::OutputQueue::Add(s, _displayLevel);
    s = std::to_string(nbCachePoints) + " perturbation vectors";
    NOMAD::OutputQueue::Add(s, _displayLevel);
    s = std::to_string(trySet.size()) + " projection candidates";
    NOMAD::OutputQueue::Add(s, _displayLevel);

    AddOutputInfo(subStepName, false, true);

    // Greedy selection
    const size_t p = trySet.size();
    std::vector<bool> keep;
    keep.resize(p);

    if (p <= _nbProjTrial)
    {
        // Greedy selection not needed - keep all
        for (size_t i = 0; i < p; i++)
        {
            keep[i] = true;
        }
    }
    else
    {
        // DO the greedy selection
        doGreedySelection(oraclePoint, trySet, keep);
    }

    for (auto xTry : trySet)
    {
        insertTrialPoint(xTry);
    }

    _indexSet.clear();
    trySet.clear();
    keep.clear();
    

    // Evaluate projection trial points
    // in the surrogate model
    // TODO Analyse from NOMAD 3 and see if we can do something similiar
    // in NOMAD 4. It may not be worth it, it seems more like an 
    // issue of sorting the points accorting to a SgtelibModel, and
    // that would be better done in the EvaluatorControl.
    //evaluateProjectionTrialPoints(trySet, ev, keep, bestEvalPoint);

    // TODO add something like SgtelibModel::checkHF() to postprocessing?

}


void NOMAD::Projection::generateTrialPoints()
{
    for (auto oraclePoint : _oraclePoints)
    {
        projectPoint(oraclePoint);
    }

}


void NOMAD::Projection::nonProjectedPoint(const NOMAD::EvalPoint& oraclePoint)
{
    insertTrialPoint(oraclePoint);
}


void NOMAD::Projection::stdProjectedPoint(const NOMAD::EvalPoint& oraclePoint)
{
    // STD projected point
    //--------------------
    NOMAD::Point xTry(*oraclePoint.getX());

    if (nullptr != _mesh)
    {
        // TODO: Use mesh and frame center from IterationUtils
        // Project the point on the mesh
        xTry = _mesh->projectOnMesh(xTry, *_frameCenter);
    }
    NOMAD::EvalPoint evalPoint(xTry);

    // TODO This code is based on NOMAD 3's Sgtelib_Model code.
    // The goal here is to evaluate points according to the SgtelibModel.
    // This may not have its place in the Projection class.
    NOMAD::CacheInterface cacheInterface(this);
    const int maxNumEval = 1;
    if (cacheInterface.smartInsert(evalPoint, maxNumEval, NOMAD::EvalType::SGTE))
    {
        insertTrialPoint(evalPoint);
    }

}


// Compute perturbation
NOMAD::Direction NOMAD::Projection::computePerturbation(const NOMAD::EvalPoint& oraclePoint,
                                                              size_t index)
{
    NOMAD::Direction perturbationDir(oraclePoint.size());

    for (size_t j = 0; j < oraclePoint.size(); j++)
    {
        NOMAD::Double dmj = _mesh->getdeltaMeshSize(j); // Get the value of delta_m

        // Inverse dmi depending on parity of index
        if (index & 1)
        {
            dmj *= -1;
        }

        // Set perturbation
        perturbationDir[j] = dmj;

        // Right shift ie: divide by 2
        index = (index >> 1);

    } // End of the construction of the perturbation

    return perturbationDir;
}


NOMAD::EvalPoint NOMAD::Projection::buildProjectionTrialPoint(const NOMAD::Point& xRef,
                                                         const NOMAD::Direction& perturbation)
{
    // Build projection trial point
    NOMAD::Point y(xRef + perturbation);

    y = _mesh->projectOnMesh(y, xRef);

    // Easier to handle an EvalPoint
    NOMAD::EvalPoint evalPoint(y);

    return evalPoint;

}


void NOMAD::Projection::doGreedySelection(const NOMAD::EvalPoint &oraclePoint, 
                                                const NOMAD::EvalPointSet& trySet,
                                                std::vector<bool>& keep)
{
    const size_t p = trySet.size();
    const size_t n = oraclePoint.size();
    std::string s;

    int inew;
    double * xnew_d;
    double * xref_d = new double [n];
    for (size_t j = 0 ; j < n ; j++)
    {
        xref_d[j] = oraclePoint[j].todouble();
    }

    // Convert set of trial candidates into an array
    //--------------------------------
    double ** X = new double * [p];
    size_t i = 0;
    for (auto xTry : trySet)
    {
        X[i] = new double [n];
        for (size_t j = 0; j < n; j++)
        {
            X[i][j] = xTry[j].todouble();
        }

        i++;
    }

    // Distance to xref
    double * Dr  = new double [p];
    // Distance to the set of selected (or kept) points
    double * Ds  = new double [p];
    double * Ds2 = new double [p]; // Square of Ds

    // First selected point
    inew = 0;
    xnew_d = X[inew];
    double lambda = 3;

    // Buffer for dxj
    double dxj;


    // Initialise the distances
    //--------------------------------------
    for (i = 0 ; i < p ; i++)
    {
        // Compute distance between each point of the set and the point xref
        // and also between each point of the set and the selected point xnew
        Dr[i] = 0;
        Ds[i] = 0;
        for (size_t j = 0 ; j < n ; j++ )
        {
            double dmj = _mesh->getdeltaMeshSize(j).todouble();
            dxj = (xref_d[j]-X[i][j])/dmj;
            Dr[i] += dxj*dxj;
            dxj = (xnew_d[j]-X[i][j])/dmj;
            Ds[i] += dxj*dxj;
        }
        Dr[i]  = sqrt(Dr[i]);
        Ds2[i] = Ds[i];
        Ds[i]  = sqrt(Ds[i]);
        // Init keep
        keep[i] = false;
    }

    // Note that we selected the first point
    size_t nbKeep = 1;
    keep[inew] = true;

    // Greedy selection
    //---------------------------
    while (nbKeep < _nbProjTrial)
    {
        // Find the index that maximizes Ds-lambda*Dr
        double Q_max = -INF;
        inew = 1;
        for (i = 0; i < p; i++)
        {
            // Q is the selection criteria: Q = Ds-lambda*Dr
            double Q = Ds[i]-lambda*Dr[i];
            if ( Q > Q_max )
            {
                inew = i;
                Q_max = Q;
            }
        }

        if ( Ds[inew] == 0 )
        {
            // If the point is already in the set, then reduce lambda
            lambda *= 0.9;
            if (lambda < 1e-6)
            {
                break;
            }
        }
        else
        {
            // Otherwise, add the point to the set
            keep[inew] = true;
            nbKeep++;
            xnew_d = X[inew];
            // Update its distance to the set
            Ds[inew] = 0;
            // Update the other distances to the set
            for (i = 0; i < p; i++)
            {
                if (Ds[i] > 0)
                {
                    // Compute distance between each point of the set and the point xref
                    // and also between each point of the set and the selected point xnew
                    double d = 0;
                    for (size_t j = 0; j < n; j++)
                    {
                        double dmj = _mesh->getdeltaMeshSize(j).todouble();
                        dxj = (xnew_d[j]-X[i][j])/dmj;
                        d += dxj * dxj;
                    }
                    if (Ds2[i] > d)
                    {
                        Ds2[i] = d;
                        Ds[i] = sqrt(d);
                    }
                }
            }
        }
    } // End while


    s = std::to_string(nbKeep) + " projection candidates (after greedy selection)";
    NOMAD::OutputQueue::Add(s, _displayLevel);

    delete [] xref_d;
    delete [] Ds;
    delete [] Ds2;
    delete [] Dr;
    for (i = 0; i < p; i++)
    {
        delete [] X[i];
    }
    delete [] X;

}
// End doGreedySelection


/*
void NOMAD::Projection::evaluateProjectionTrialPoints(const NOMAD::EvalPointSet& trySet,
                                                    const NOMAD::SgtelibModelEvaluator& ev,
                                                    const std::vector<bool>& keep,
                                                    NOMAD::EvalPoint& bestEvalPoint)
{
    size_t i = 0;
    bool countEval = false;
    std::string s;
    auto bestF = bestEvalPoint.getF(evalType);
    auto bestH = bestEvalPoint.getH(evalType);

    for (auto evalPoint : trySet)
    {
        if (keep[i])
        {
            // Eval (with the same evaluator as for the model optimization)
            ev.eval_x(evalPoint, 0.0, countEval);
            _modelAlgo->checkHF(evalPoint);

            auto f = evalPoint.getF(evalType);
            auto h = evalPoint.getH(evalType);

            if ( ((h > 0) && (h < bestH)) || ( (h == 0) && (f < bestF) ) )
            {
                s = evalPoint.display() + " (new best projetion)";
                NOMAD::OutputQueue::Add(s, _displayLevel);

                bestEvalPoint = evalPoint;
                bestF = f;
                bestH = h;
            }
        }
        i++;
    }
}
*/
