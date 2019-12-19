#ifndef __NOMAD400_OIGNONPOLLMETHOD__
#define __NOMAD400_OIGNONPOLLMETHOD__

#include "../../Algos/Mads/PollMethod.hpp"

#include "../../nomad_nsbegin.hpp"


/// Oignon Poll Method
/**
 The Oignon Poll method consists in generating points from nested poll frames F^k with differents \Delta^k parameters 
 */
class OignonPollMethod final : public PollMethod
{
public:
    /// Constructor
    /**
     /param parentStep      The parent of this search step -- \b IN.
     */
    explicit OignonPollMethod(const NOMAD::Step* parentStep )
    : PollMethod( parentStep )
    {
        init();
    }
    
private:
    void init();
    
    /// Generate new points to evaluate
    /**
     /return The set of eval points.
     */
    void generateTrialPoints() override;
    
    
    bool computeRandDirOnUnitSphere(NOMAD::Direction &dir) const;

    std::list<NOMAD::Direction> householder(const NOMAD::Direction &initDir, bool completeTo2n ) const;


    std::list<NOMAD::Direction> strategicalDirections() const;

    void setPollDirections(std::list<NOMAD::Direction> &directions) const;

    size_t n;
    
};

#include "../../nomad_nsend.hpp"

#endif  //__NOMAD400_ENRICHEDPOLLMETHOD__
