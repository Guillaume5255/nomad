#include "../../Algos/Mads/MadsIteration.hpp"
#include "../../Algos/Mads/EnrichedPollMethod.hpp"
#include "../../Math/RNG.hpp"


void NOMAD::EnrichedPollMethod::init()
{
    _name = "Enriched Poll Method";
    
    n = _pbParams->getAttributeValue<size_t>("DIMENSION");

    auto enable = _runParams->getAttributeValue<bool>("ENRICHED_POLL");
    
    setEnabled(enable);
}

bool NOMAD::EnrichedPollMethod::computeRandDirOnUnitSphere(NOMAD::Direction &dir) const
{
    size_t i;
    NOMAD::Double norm, d;
    // VRM we use this value a lot, maybe do a get_dimension().

    // GL : dir is set such that ||dir||_2 = 1
     
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

std::list<NOMAD::Direction> NOMAD::EnrichedPollMethod::householder(const NOMAD::Direction &initDir, bool completeTo2n ) const
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

std::list<NOMAD::Direction> NOMAD::EnrichedPollMethod::strategicalDirections() const
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
    int nb2NblockOfDir;
	if(_runParams->getAttributeValue<bool>("DYNAMIC_POLL")){
		if(_runParams->getAttributeValue<std::string>("INTENSIFICATION_FACTOR") == "EXPONENTIAL" )
			nb2NblockOfDir = std::min(_runParams->getAttributeValue<int>("NUMBER_OF_2N_BLOCK"), (int)std::pow(2.0, (float)nbOfPreviousFailure));
		else
			nb2NblockOfDir = std::min(_runParams->getAttributeValue<int>("NUMBER_OF_2N_BLOCK"), (int)nbOfPreviousFailure+(int)1);
	}
	else{ 
		nb2NblockOfDir = _runParams->getAttributeValue<int>("NUMBER_OF_2N_BLOCK");
	}
	AddOutputInfo("Number of 2n blocks generated : "+std::to_string(nb2NblockOfDir), NOMAD::OutputLevel::LEVEL_VERY_HIGH);

    for(int j=0;j <nb2NblockOfDir; j++){
        {
            dirComputed = dirComputed && computeRandDirOnUnitSphere(v);
            if (!dirComputed)
            {
                std::string err("Poll: setPollDirections: Cannot compute a random direction on unit sphere");
                throw NOMAD::Exception(__FILE__, __LINE__, err);
            }
            pollDirs.merge(householder(v,true));
        }
    }
    /*
    else
    {
        std::string err("Poll: setPollDirections: this strategy is not yet implemented ");
        throw NOMAD::Exception(__FILE__, __LINE__, err);
    }*/
    return pollDirs;
}

// Generate poll directions
void NOMAD::EnrichedPollMethod::setPollDirections(std::list<NOMAD::Direction> &directions) const
{
    directions.clear();
    
    std::list<NOMAD::Direction> pollDirs = strategicalDirections(); // list of new directions, not necessarly unitary
    
    // Scale the directions and project on the mesh
    std::list<NOMAD::Direction>::const_iterator itDir;
    int k = 1;
    std::shared_ptr<NOMAD::MeshBase> mesh = getIterationMesh();
    if (nullptr == mesh)
    {
        std::string err("Poll: setPollDirections: Iteration or Mesh not found.");
        throw NOMAD::Exception(__FILE__, __LINE__, err);
    }
    NOMAD::Double frameLB = _runParams->getAttributeValue<NOMAD::Double>("FRAME_LB");
    NOMAD::Double frameUB = _runParams->getAttributeValue<NOMAD::Double>("FRAME_UB");
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

        NOMAD::Double FrameCoeff(NOMAD::RNG::rand(frameLB.todouble(),frameUB.todouble()));

        for (size_t i = 0; i < n; ++i)
        {
            // Scaling and projection on the mesh
            (*pd)[i] = mesh->scaleAndProjectOnMesh(i, ((*itDir)[i] / infiniteNorm)*FrameCoeff);
            // this projection is made as in the article about grannular variables so every dirction is on the frame when no coeff multiplicates infinite norm
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
void NOMAD::EnrichedPollMethod::generateTrialPoints()
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


