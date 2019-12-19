#include "../../Algos/EvcInterface.hpp"

#include "../../Algos/Mads/ClassicalPollMethod.hpp"

#include "../../Math/RNG.hpp"

void NOMAD::ClassicalPollMethod::init()
{
    _name = "Classical Poll Method";

    auto enable = _runParams->getAttributeValue<bool>("CLASSICAL_POLL");
    
    setEnabled(enable);
}

// Generate poll directions
void NOMAD::ClassicalPollMethod::setPollDirections(std::list<NOMAD::Direction> &directions) const
{
    directions.clear();
    size_t n = _pbParams->getAttributeValue<size_t>("DIMENSION");

    NOMAD::Direction dirUnit(n, 0.0);
    bool dirComputed = computeDirOnUnitSphere(dirUnit);
    if (!dirComputed)
    {
        std::string err("ClassicalPoll: setPollDirections: Cannot compute a random direction on unit sphere");
        throw NOMAD::Exception(__FILE__, __LINE__, err);
    }

    // Ortho MADS 2n
    // Householder Matrix
    NOMAD::Direction** H = new NOMAD::Direction*[2*n];
    std::list<NOMAD::Direction> dirsUnit;

    // Ordering D_k alternates Hk and -Hk instead of [H_k -H_k]
    // VRM a revoir avec le livre, pour les notations.
    for (size_t i = 0; i < n; ++i)
    {
        dirsUnit.push_back(NOMAD::Direction(n, 0.0));
        H[i]   = &(*(--dirsUnit.end())); // VRM I don't know, I just work here
        dirsUnit.push_back(NOMAD::Direction(n, 0.0));
        H[i+n] = &(*(--dirsUnit.end()));
    }

    // Householder transformations on the 2n directions on a unit n-sphere
    // VRM: Revoir la theorie pour Householder et pour Poll.
    householder(dirUnit, true, H);
    delete [] H;

    // Scale the directions and project on the mesh
    std::list<NOMAD::Direction>::const_iterator itDir;
    int k = 1;
    std::shared_ptr<NOMAD::MeshBase> mesh = getIterationMesh();
    if (nullptr == mesh)
    {
        std::string err("ClassicalPoll: setPollDirections: Iteration or Mesh not found.");
        throw NOMAD::Exception(__FILE__, __LINE__, err);
    }

    for (itDir = dirsUnit.begin(); itDir != dirsUnit.end(); ++itDir, ++k)
    {
        directions.push_back(NOMAD::Direction(n, 0.0));
        NOMAD::Direction* pd = &(*(--directions.end()));    // VRM he's doing it again

        // Compute infinite norm for direction pointed by itDir.
        NOMAD::Double infiniteNorm = (*itDir).infiniteNorm();
        if (0 == infiniteNorm)
        {
            std::string err("ClassicalPoll: setPollDirections: Cannot handle an infinite norm of zero");
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

    dirsUnit.clear();
}


// Generate new points to evaluate
void NOMAD::ClassicalPollMethod::generateTrialPoints()
{
    AddOutputInfo("Generate points for " + _name, true, false);
    
    // Creation of the poll directions
    std::list<NOMAD::Direction> directions;
    setPollDirections(directions);
    
    size_t n = _pbParams->getAttributeValue<size_t>("DIMENSION");
    
    // We need a poll center to start with.
    auto pollCenter = getIterationFrameCenter();
    if (nullptr == pollCenter || !pollCenter->ArrayOfDouble::isDefined() || pollCenter->size() != n)
    {
        std::string err("NOMAD::ClassicalPoll::generateTrialPoints: invalid poll center: ");
        if (nullptr != pollCenter)
        {
            err += pollCenter->display();
        }
        throw NOMAD::Exception(__FILE__, __LINE__, err);
    }
    
    AddOutputDebug("Poll center: " + pollCenter->display());
    
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



// VRM only used in Poll, but maybe should be in a more general class.
/*--------------------------------------------------*/
/*  Compute a random direction on a unit N-Sphere   */
/*  See http://en.wikipedia.org/wiki/N-sphere       */
/*--------------------------------------------------*/

bool NOMAD::ClassicalPollMethod::computeDirOnUnitSphere(NOMAD::Direction &randomDir) const
{
    size_t i;
    NOMAD::Double norm, d;
    // VRM we use this value a lot, maybe do a get_dimension().
    size_t n = _pbParams->getAttributeValue<size_t>("DIMENSION");

    for (i = 0; i < n; ++i)
    {
        randomDir[i] = NOMAD::RNG::normalRand(0,1);
    }

    norm = randomDir.norm();

    if (0 == norm)
    {
        // it failed
        return false;
    }

    for (i = 0; i < n; ++i)
    {
        randomDir[i] /= norm;
    }

    // it worked
    return true;
}


// VRM is this used only by Poll? Should it (still) be moved to
// some more general class? in the Math folder?
/*----------------------------------------------------------------*/
/*  - Householder transformation to generate _nc directions from  */
/*    a given direction.                                          */
/*  - Compute also H[i+nc] = -H[i] (completion to 2n directions). */
/*  - Private method.                                              */
/*----------------------------------------------------------------*/
void NOMAD::ClassicalPollMethod::householder(const NOMAD::Direction &dir,
                              bool completeTo2n,
                              NOMAD::Direction ** H) const
{
    size_t n = _pbParams->getAttributeValue<size_t>("DIMENSION");

    const NOMAD::Double norm2 = dir.norm();
    NOMAD::Double v, h2i;

    for (size_t i = 0 ; i < n ; ++i)
    {
        h2i = 2 * dir[i];
        for (size_t j = 0 ; j < n ; ++j)
        {
            // H[i]:
            (*H[i])[j] = v = (i == j) ? norm2 - h2i * dir[j] : - h2i * dir[j];

            // -H[i]:
            if ( completeTo2n )
            {
                (*H[i+n])[j] = -v;
            }
        }
    }
}