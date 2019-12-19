#ifndef __NOMAD400_ENRICHEDPOLLMETHOD__
#define __NOMAD400_ENRICHEDPOLLMETHOD__

#include "../../Algos/Mads/PollMethod.hpp"

#include "../../nomad_nsbegin.hpp"


/// Enriched Poll
/**
 The Enriched Poll method consists in generating more poll directions : 4n, 8n, 12n instead of 2n
 that can also be inside the frame.
 */
class EnrichedPollMethod final : public PollMethod
{
public:
    /// Constructor
    /**
     /param parentStep      The parent of this search step -- \b IN.
     */
    explicit EnrichedPollMethod(const NOMAD::Step* parentStep )
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

    /// \note Multiplicative factor TODO
    
};

#include "../../nomad_nsend.hpp"

#endif  //__NOMAD400_ENRICHEDPOLLMETHOD__
