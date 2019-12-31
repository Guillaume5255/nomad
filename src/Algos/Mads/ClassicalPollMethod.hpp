#ifndef __NOMAD400_CLASSICALPOLL__
#define __NOMAD400_CLASSICALPOLL__

#include "../../Algos/Mads/PollMethod.hpp"

#include "../../nomad_nsbegin.hpp"


// \class ClassicalPoll for Step 3. of MADS algorithm.
class ClassicalPollMethod final : public PollMethod
{
public:
    /// Constructor
    /**
     \param parentStep The parent of this poll step
     */
    explicit ClassicalPollMethod(const Step* parentStep )
    : PollMethod(parentStep)
    {
        init();
    }
    ~ClassicalPollMethod() {}
    
private:
    void init();

    /// Generate new points to evaluate
    void generateTrialPoints() override ;

    /*------------------------------*/
    /* Private methods used by poll */
    /*------------------------------*/
    
    /// Generate poll directions
    /**
     /param directions  The directions obtained for this poll -- \b OUT.
     */
    void setPollDirections(std::list<NOMAD::Direction> &directions) const;


    /*------------------------*/
    /* Private helper methods */
    /*------------------------*/
    bool computeDirOnUnitSphere(NOMAD::Direction &randomDir) const;

    void householder(const NOMAD::Direction &dir, bool completeTo2n, NOMAD::Direction ** H) const;


};

#include "../../nomad_nsend.hpp"

#endif