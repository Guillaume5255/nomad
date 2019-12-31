#ifndef __NOMAD400_MULTIPOLLMETHOD__
#define __NOMAD400_MULTIPOLLMETHOD__

#include "../../Algos/Mads/PollMethod.hpp"

#include "../../nomad_nsbegin.hpp"


/// Multi Poll
/**
 The Multi Poll method consists in generating the second neighbour of the poll points.
 */
class MultiPollMethod  final : public PollMethod
{
public:
    /// Constructor
    /**
     /param parentStep      The parent of this search step -- \b IN.
     */
    explicit MultiPollMethod(const NOMAD::Step* parentStep )
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


    void multiPollDir(std::list<NOMAD::Direction> &primaryPollDirs) const;

    std::list<NOMAD::Direction> strategicalDirections() const;

    void setPollDirections(std::list<NOMAD::Direction> &directions) const;

    size_t n;

    /// \note Multiplicative factor TODO
    
};

#include "../../nomad_nsend.hpp"

#endif // __NOMAD400_MULTIPOLLMETHOD__
