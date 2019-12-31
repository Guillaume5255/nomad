#ifndef __NOMAD400_POLL__
#define __NOMAD400_POLL__

#include "../../Algos/Mads/PollMethod.hpp"

#include "../../Algos/Step.hpp"

#include "../../nomad_nsbegin.hpp"

/// Class for the poll (step 3) of MADS algorithm.
/**
 Generate the trial points (Poll::startImp), launch evaluation (Poll::runImp) and postprocecssing (Poll::endImp).
 */
class Poll final : public Step , public MadsIterationUtils
{
private:
    std::vector<std::shared_ptr<NOMAD::PollMethod>> _pollMethods;

public:
    /// Constructor
    /**
     /param parentStep      The parent of this poll step -- \b IN.
     */
    explicit Poll(const NOMAD::Step* parentStep )
      : Step( parentStep ),
        MadsIterationUtils( parentStep ),
        _pollMethods()
    {
        init();
    }
    virtual ~Poll() {}

    // Generate new points to evaluate. Use all enabled search methods.

    /**
     The trial points are obtained by:
        - adding poll directions (Poll::setPollDirections) to the poll center (frame center).
        - snaping points (and directions) to bounds.
        - projecting points on mesh.
     */
    void generateTrialPoints() override ;
    
private:
    /// Helper for constructor
    void init();

    
    // Identify if there is at least one poll enabled.
    bool isEnabled() const;

	
    /// Implementation for start tasks for MADS poll.
    /**
     Call to generate trial points and test for mesh precision
     */
    virtual void startImp() override ;
    
    /// Implementation for run tasks for MADS poll.
    /**
     Start trial points evaluation.
     \return Flag \c true if found better solution \c false otherwise.
     */
    virtual bool runImp() override;
    
    /// Implementation for end tasks for MADS poll.
    /**
     Call the IterationUtils::postProcessing of the points.
     */
    virtual void endImp() override ;
    
};

#include "../../nomad_nsend.hpp"

#endif // __NOMAD400_SEARCH__

