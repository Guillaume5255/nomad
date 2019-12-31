
#include "../../Algos/Mads/Poll.hpp"
#include "../../Algos/Mads/ClassicalPollMethod.hpp"
#include "../../Algos/Mads/MultiPollMethod.hpp"
#include "../../Algos/Mads/EnrichedPollMethod.hpp"
#include "../../Algos/Mads/OignonPollMethod.hpp"

void NOMAD::Poll::init()
{
    _name = "Poll";
    verifyParentNotNull();

    auto cPoll = std::make_shared<NOMAD::ClassicalPollMethod>(this );
    auto mPoll = std::make_shared<NOMAD::MultiPollMethod>(this );
    auto ePoll = std::make_shared<NOMAD::EnrichedPollMethod>(this );
    auto oPoll = std::make_shared<NOMAD::OignonPollMethod>(this );

    // The poll methods will be executed in the same order
    // as they are inserted.
    _pollMethods.push_back(cPoll);
    _pollMethods.push_back(mPoll);
    _pollMethods.push_back(ePoll);
    _pollMethods.push_back(oPoll);


}


void NOMAD::Poll::startImp()
{
    verifyGenerateAllPointsBeforeEval("Poll::start()", false);

    if (!isEnabled())
    {
        // Early out
        return;
    }
    
    // Generate the points from all the enabled search methods before starting evaluations
    if ( _runParams->getAttributeValue<bool>("GENERATE_ALL_POINTS_BEFORE_EVAL") )
        generateTrialPoints();
}


bool NOMAD::Poll::runImp()
{
    bool foundBetter = false;
    std::string s;

    // This function should be called only when trial points are generated for each search method separately and evaluated.
    verifyGenerateAllPointsBeforeEval(__PRETTY_FUNCTION__, false);

    if (!isEnabled())
    {
        // Early out --> no found better!
        AddOutputDebug("Search method is disabled. Early out.");
        return false;
    }


    NOMAD::SuccessType bestSuccessYet = NOMAD::SuccessType::NOT_EVALUATED;
    NOMAD::SuccessType success = NOMAD::SuccessType::NOT_EVALUATED;

    // Go through all search methods until we get a success.
    s = "Going through all poll methods until we get a success";
    AddOutputDebug(s);
    for (size_t i = 0; !foundBetter && i < _pollMethods.size(); i++)
    {
        auto pollMethod = _pollMethods[i];
        bool enabled = pollMethod->isEnabled();
        s = "Search method " + pollMethod->getName() + (enabled ? " is enabled" : " not enabled");
        AddOutputDebug(s);
        if (!enabled) { continue; }
        pollMethod->start();
        foundBetter = pollMethod->run();
        success = pollMethod->getSuccessType();
        if (success > bestSuccessYet)
        {
            bestSuccessYet = success;
        }
        pollMethod->end();

        if (foundBetter)
        {
            // Do not go through the other poll methods if we found
            // an improving solution.
            s = pollMethod->getName();
            s += " found an improving solution. Stop reason: ";
            s += _stopReasons->getStopReasonAsString() ;

            AddOutputInfo(s);
            break;
        }
    }

    setSuccessType(bestSuccessYet);

    return foundBetter;
}

void NOMAD::Poll::endImp()
{
    verifyGenerateAllPointsBeforeEval(__PRETTY_FUNCTION__, false);

    if (!isEnabled())
    {
        // Early out
        return;
    }

    // Need to reset the EvalStopReason if a sub optimization is used during Search and the max bb is reached for this sub optimization
    if (_stopReasons->testIf(NOMAD::EvalStopType::LAP_MAX_BB_EVAL_REACHED))
    {
        _stopReasons->set(NOMAD::EvalStopType::STARTED);
    }

}

// Generate trial points for ALL enabled search methods.
// To be used only when parameter GENERATE_ALL_POINTS_BEFORE_EVAL is true.
void NOMAD::Poll::generateTrialPoints()
{

    verifyGenerateAllPointsBeforeEval("Poll::generateTrialPoints()", true);

    for (auto pollMethod : _pollMethods)
    {
        if (pollMethod->isEnabled())
        {
            pollMethod->generateTrialPoints();
            
            // NB verifyPointsAreOnMesh and updatePointsWithFrameCenter are
            // done here, so that if an user adds a new search method, they
            // will not have to think about adding these verifications.
            pollMethod->verifyPointsAreOnMesh(getName());
            pollMethod->updatePointsWithFrameCenter();
            
            auto pollMethodPoints = pollMethod->getTrialPoints();

            for (auto point : pollMethodPoints)
            {
                // NB trialPoints includes points from multiple poll methods.
                // VRM Probably we should use some move operators instead of
                // copying EvalPoints.
                insertTrialPoint(point);
            }
        }
    }
}


bool NOMAD::Poll::isEnabled() const
{
    bool pollEnabled = false;
    if (_pollMethods.size() > 0)
    {
        for (auto pollMethod : _pollMethods)
        {
            if (pollMethod->isEnabled())
            {
                pollEnabled = true;
                break;
            }
        }
    }

    return pollEnabled;
}



