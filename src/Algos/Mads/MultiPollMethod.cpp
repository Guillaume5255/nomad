/**
 \file   MultiPollMethod.cpp
 \brief  Multi Poll (implementation)
 \author Guillaume Lameynardie
 \date   2019-11-07
 */
#include "../../Algos/Mads/MadsIteration.hpp"
#include "../../Algos/Mads/MultiPollMethod.hpp"
#include "../../Math/RNG.hpp"


void NOMAD::MultiPollMethod::init()
{
    _name = "Multi Poll Method";
    
    n = _pbParams->getAttributeValue<size_t>("DIMENSION");

    auto enable = _runParams->getAttributeValue<bool>("MULTI_POLL");
    
    setEnabled(enable);
}

bool NOMAD::MultiPollMethod::computeRandDirOnUnitSphere(NOMAD::Direction &dir) const
{
    size_t i;
    NOMAD::Double norm, d;
    // VRM we use this value a lot, maybe do a get_dimension().
     
    for (i = 0; i < n; ++i)
    {
        dir[i] = NOMAD::RNG::normalRand(0,1);
    }

    norm = dir.norm();

    if (0 == norm)
    {
        // it failed
        return false;
    }

    for (i = 0; i < n; ++i)
    {
        dir[i] /= norm;
    }
    // it worked
    return true;
}

std::list<NOMAD::Direction> NOMAD::MultiPollMethod::householder(const NOMAD::Direction &initDir, bool completeTo2n ) const
{
    std::list<NOMAD::Direction> positiveSpanningSet;

    const NOMAD::Double norm2 = (initDir.norm() == 0.0) ? 1.0 : initDir.norm();

     
    for (size_t i = 0 ; i < n ; ++i)
    {
        NOMAD::Direction pollDir(n,0.0);
        for (size_t j = 0 ; j < n ; ++j)
        {
            pollDir[j] = (i==j) ?  norm2 - 2*initDir[i]*initDir[j] :  - 2*initDir[i]*initDir[j];
        }
        positiveSpanningSet.push_back(pollDir);

        if(completeTo2n)
        {
            for (size_t j = 0 ; j < n ; ++j)
            {
                pollDir[j]= - pollDir[j];
            }
            positiveSpanningSet.push_back(pollDir);
        }
    }
    return positiveSpanningSet;
}


void NOMAD::MultiPollMethod::multiPollDir(std::list<NOMAD::Direction> &primaryPollDirs) const{
    std::list<NOMAD::Direction> secondaryPollDirs;
    std::list<NOMAD::Direction> allSecondaryPollDirs;

    for (auto ppd = primaryPollDirs.begin(); ppd != primaryPollDirs.end(); ppd++){
        secondaryPollDirs = householder(*ppd,true);
        for(auto spd = secondaryPollDirs.begin(); spd != secondaryPollDirs.end(); spd++){
            for(size_t i = 0; i<n; i++){
                (*spd)[i] = (*spd)[i] + (*ppd)[i];
            }
        }
        allSecondaryPollDirs.merge(secondaryPollDirs);
    }
    primaryPollDirs.merge(allSecondaryPollDirs);
    
}

std::list<NOMAD::Direction> NOMAD::MultiPollMethod::strategicalDirections() const
{

    NOMAD::Direction v(n, 0.0); // in the householder definition : H = I-2v*v^t
    
    std::list<NOMAD::Direction> pollDirs;

    bool dirComputed = false;
    dirComputed = computeRandDirOnUnitSphere(v);
    if (!dirComputed)
    {   
        std::string err("Poll: setPollDirections: Cannot compute a random direction on unit sphere");
        throw NOMAD::Exception(__FILE__, __LINE__, err);
    }
    pollDirs = householder(v,true);
    multiPollDir(pollDirs);
    
    /*
    else
    {
        std::string err("Poll: setPollDirections: this strategy is not yet implemented ");
        throw NOMAD::Exception(__FILE__, __LINE__, err);
    }*/
    return pollDirs;
}

// Generate poll directions
void NOMAD::MultiPollMethod::setPollDirections(std::list<NOMAD::Direction> &directions) const
{
    directions.clear();
    
    std::list<NOMAD::Direction> pollDirs = strategicalDirections() ; // list of new directions, not necessarly unitary
    
    // Scale the directions and project on the mesh
    std::list<NOMAD::Direction>::const_iterator itDir;
    int k = 1;
    std::shared_ptr<NOMAD::MeshBase> mesh = getIterationMesh();
    if (nullptr == mesh)
    {
        std::string err("Poll: setPollDirections: Iteration or Mesh not found.");
        throw NOMAD::Exception(__FILE__, __LINE__, err);
    }
    for (itDir = pollDirs.begin(); itDir != pollDirs.end(); ++itDir, ++k)
    {
        directions.push_back(NOMAD::Direction(n, 0.0));
        NOMAD::Direction* pd = &(*(--directions.end()));    // VRM he's doing it again

        // Compute infinite norm for direction pointed by itDir.
        NOMAD::Double infiniteNorm = (*itDir).infiniteNorm();
        if (0 == infiniteNorm)
        {
            std::string err("Poll: setPollDirections: Cannot handle an infinite norm of zero");
            throw NOMAD::Exception(__FILE__, __LINE__, err);
        }

        for (size_t i = 0; i < n; ++i)
        {
            // Scaling and projection on the mesh
            (*pd)[i] = mesh->scaleAndProjectOnMesh(i, (*itDir)[i] / infiniteNorm);
        }
    }

    std::list<NOMAD::Direction>::const_iterator it;
    for (it = directions.begin(); it != directions.end(); ++it)
    {
        AddOutputDebug("Poll direction: " + it->display());
    }

    pollDirs.clear();
}

// Generate new points to evaluate
void NOMAD::MultiPollMethod::generateTrialPoints()
{
    AddOutputInfo("Generate points for " + _name, true, false);

    NOMAD::EvalPointSet trialPoints;

    // We need a frame center to start with.
    auto pollCenter = getIterationFrameCenter();
    if (nullptr == pollCenter || !pollCenter->ArrayOfDouble::isDefined() || pollCenter->size() != n)
    {
        std::string err("NOMAD::Poll::generateTrialPoints: invalid poll center: ");
        if (nullptr != pollCenter)
        {
            err += pollCenter->display();
        }
        throw NOMAD::Exception(__FILE__, __LINE__, err);
    }

    AddOutputDebug("Poll center: " + pollCenter->display());

    //addDirectionToPollCenter(*pollCenter,trialPoints); // creates y_j = x_k + d_j and store them in the set trial points 
    std::list<NOMAD::Direction> directions;
    setPollDirections(directions);
    
    for (std::list<NOMAD::Direction>::iterator it = directions.begin(); it != directions.end() ; ++it)
    {
        NOMAD::Point pt(n);
        
        // pt = poll center + direction
        for (size_t i = 0 ; i < n ; ++i )
        {
            pt[i] = (*pollCenter)[i] + (*it)[i];
        }
        
        // Snap the points and the corresponding direction to the bounds
        pt.snapToBounds(_pbParams->getAttributeValue<NOMAD::ArrayOfDouble>("LOWER_BOUND"),
                        _pbParams->getAttributeValue<NOMAD::ArrayOfDouble>("UPPER_BOUND"),
                        *pollCenter,
                        getIterationMesh()->getdeltaMeshSize());

        if (!getIterationMesh()->verifyPointIsOnMesh(pt, *pollCenter))
        {
            auto ptdebug = pt;
            pt = getIterationMesh()->projectOnMesh(pt, *pollCenter);
            if (ptdebug != pt)
            {
                AddOutputWarning("Warning: " + ptdebug.display() + " projected to " + pt.display());
            }
        }

        if (pt != *pollCenter->getX())
        {
            // New EvalPoint to be evaluated.
            // Add it to the list.
            bool inserted = insertTrialPoint(NOMAD::EvalPoint(pt));
            
            std::string s = "Generated point";
            s += (inserted) ? ": " : " not inserted: ";
            s += pt.display();
            AddOutputInfo(s);
        }
    }
    verifyPointsAreOnMesh(getName());
    updatePointsWithFrameCenter();
    
    AddOutputInfo("Generated " + NOMAD::itos(getTrialPointsCount()) + " points");
    AddOutputInfo("Generate points for " + _name, false, true);
    
}


