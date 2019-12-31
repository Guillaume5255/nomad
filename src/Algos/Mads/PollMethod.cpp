
#include "../../Algos/EvcInterface.hpp"

#include "../../Algos/Mads/PollMethod.hpp"

void NOMAD::PollMethod::init()
{
    // A search method must have a parent
    verifyParentNotNull();
}


void NOMAD::PollMethod::startImp()
{
    verifyGenerateAllPointsBeforeEval(__PRETTY_FUNCTION__, false);

    if ( ! _stopReasons->checkTerminate() )
    {
        // Create EvalPoints
        generateTrialPoints();
        // NB verifyPointsAreOnMesh and updatePointsWithPollCenter are
        // done here, so that if an user adds a new poll method, they
        // will not have to think about adding these verifications.
        verifyPointsAreOnMesh(getName());
        updatePointsWithFrameCenter();
    }
}


bool NOMAD::PollMethod::runImp()
{
    verifyGenerateAllPointsBeforeEval(__PRETTY_FUNCTION__, false);

    bool foundBetter = false;

    if ( ! _stopReasons->checkTerminate() )
    {
        // Show more information in the form of an AlgoComment.
        NOMAD::MainStep::setAlgoComment(getComment());

        foundBetter = evalTrialPoints(this);
        NOMAD::MainStep::resetPreviousAlgoComment();

    }
    return foundBetter;

}


void NOMAD::PollMethod::endImp()
{
    // Compute hMax and update Barrier.
    postProcessing(getEvalType());

}
