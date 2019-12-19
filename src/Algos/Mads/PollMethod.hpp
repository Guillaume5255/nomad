#ifndef __NOMAD400_POLLMETHOD__
#define __NOMAD400_POLLMETHOD__

#include "../../Algos/Mads/MadsIterationUtils.hpp"

#include "../../nomad_nsbegin.hpp"

// \class PollMethod: Run by Poll. Step of MADS algorithm.
class PollMethod: public Step , public MadsIterationUtils
{
private:
    // Should this poll method be used? Modified by parameters.
    bool _enabled;

public:
    /// Constructor
    /**
     /param parentStep      The parent of this search step -- \b IN.
     */
    explicit PollMethod( const NOMAD::Step* parentStep )
      : Step( parentStep ),
        MadsIterationUtils ( parentStep ),
        _enabled(true)
    {
        init();
    }

    // Get / Set
    bool isEnabled() const { return _enabled; }
    void setEnabled(const bool enabled) { _enabled = enabled; }

    // Step methods
    virtual void startImp() override ;
    virtual bool runImp() override ;
    virtual void endImp() override ;

protected:
    void init();

};

#include "../../nomad_nsend.hpp"

#endif // __NOMAD400_SEARCHMETHOD__

