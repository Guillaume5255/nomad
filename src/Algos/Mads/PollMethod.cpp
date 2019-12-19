
#include "../../Algos/EvcInterface.hpp"

#include "../../Algos/Mads/PollMethod.hpp"

void NOMAD::PollMethod::init()
{
    // A search method must have a parent
    verifyParentNotNull();
}


void NOMAD::PollMethod::startImp()
{
    verifyGenerateAllPointsBeforeEval("PollMethod::start()", false);

    if ( ! _stopReasons->checkTerminate() )
    {
        // Create EvalPoints
        generateTrialPoints();
        // NB verifyPointsAreOnMesh and updatePointsWithPollCenter are
        // done here, so that if an user adds a new search method, they
        // will not have to think about adding these verifications.
        verifyPointsAreOnMesh(getName());
        updatePointsWithFrameCenter();
    }
}


bool NOMAD::PollMethod::runImp()
{
    verifyGenerateAllPointsBeforeEval("PollMethod::run()", false);

    bool foundBetter = false;

    if ( ! _stopReasons->checkTerminate() )
    {
        foundBetter = evalTrialPoints(this);
    }
    return foundBetter;

}


void NOMAD::PollMethod::endImp()
{
    // Compute hMax and update Barrier.
    postProcessing(getEvalType());

    Step::end();
}
