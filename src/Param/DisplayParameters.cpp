/*---------------------------------------------------------------------------------*/
/*  NOMAD - Nonlinear Optimization by Mesh Adaptive Direct Search -                */
/*                                                                                 */
/*  NOMAD - Version 4.0.0 has been created by                                      */
/*                 Viviane Rochon Montplaisir  - Polytechnique Montreal            */
/*                 Christophe Tribes           - Polytechnique Montreal            */
/*                                                                                 */
/*  The copyright of NOMAD - version 4.0.0 is owned by                             */
/*                 Sebastien Le Digabel        - Polytechnique Montreal            */
/*                 Viviane Rochon Montplaisir  - Polytechnique Montreal            */
/*                 Christophe Tribes           - Polytechnique Montreal            */
/*                                                                                 */
/*  NOMAD v4 has been funded by Rio Tinto, Hydro-Québec, NSERC (Natural Science    */
/*  and Engineering Research Council of Canada), INOVEE (Innovation en Energie     */
/*  Electrique and IVADO (The Institute for Data Valorization)                     */
/*                                                                                 */
/*  NOMAD v3 was created and developed by Charles Audet, Sebastien Le Digabel,     */
/*  Christophe Tribes and Viviane Rochon Montplaisir and was funded by AFOSR       */
/*  and Exxon Mobil.                                                               */
/*                                                                                 */
/*  NOMAD v1 and v2 were created and developed by Mark Abramson, Charles Audet,    */
/*  Gilles Couture, and John E. Dennis Jr., and were funded by AFOSR and           */
/*  Exxon Mobil.                                                                   */
/*                                                                                 */
/*  Contact information:                                                           */
/*    Polytechnique Montreal - GERAD                                               */
/*    C.P. 6079, Succ. Centre-ville, Montreal (Quebec) H3C 3A7 Canada              */
/*    e-mail: nomad@gerad.ca                                                       */
/*    phone : 1-514-340-6053 #6928                                                 */
/*    fax   : 1-514-340-5665                                                       */
/*                                                                                 */
/*  This program is free software: you can redistribute it and/or modify it        */
/*  under the terms of the GNU Lesser General Public License as published by       */
/*  the Free Software Foundation, either version 3 of the License, or (at your     */
/*  option) any later version.                                                     */
/*                                                                                 */
/*  This program is distributed in the hope that it will be useful, but WITHOUT    */
/*  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or          */
/*  FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License    */
/*  for more details.                                                              */
/*                                                                                 */
/*  You should have received a copy of the GNU Lesser General Public License       */
/*  along with this program. If not, see <http://www.gnu.org/licenses/>.           */
/*                                                                                 */
/*  You can find information on the NOMAD software at www.gerad.ca/nomad           */
/*---------------------------------------------------------------------------------*/

#include <iomanip>  // For std::setprecision
#include "../Math/RNG.hpp"
#include "../Param/DisplayParameters.hpp"


/*----------------------------------------*/
/*         initializations (private)      */
/*----------------------------------------*/
void NOMAD::DisplayParameters::init()
{
    _typeName = "Display";

    try
    {
        #include "../Attribute/displayAttributesDefinition.hpp"
        registerAttributes( _definition );
        
        // Note: we cannot call checkAndComply() here, the default values
        // are not valid, for instance DIMENSION, X0, etc.
        
    }
    catch ( NOMAD::Exception & e)
    {
        std::string errorMsg = "Attribute registration failed: ";
        errorMsg += e.what();
        throw NOMAD::Exception(__FILE__,__LINE__, errorMsg);
    }
    
}

/*----------------------------------------*/
/*            check the parameters        */
/*----------------------------------------*/
void NOMAD::DisplayParameters::checkAndComply(
                    const std::shared_ptr<NOMAD::RunParameters> &runParams,
                    const std::shared_ptr<NOMAD::PbParameters> &pbParams)
{

    checkInfo();
    
    if (!toBeChecked())
    {
        // Early out
        return;
    }
    
    // Pb params must be checked before accessing its value
    size_t n = pbParams->getAttributeValue<size_t>("DIMENSION");
    if (n == 0)
    {
        throw NOMAD::Exception(__FILE__,__LINE__, "Parameters check: DIMENSION must be positive" );
    }

    
    // SOL_FORMAT (internal)
    auto solFormat = getAttributeValueProtected<NOMAD::ArrayOfDouble>("SOL_FORMAT",false);
    if ( !solFormat.isDefined() )
    {
        solFormat.reset(n, NOMAD::DISPLAY_PRECISION_STD);
        setAttributeValue("SOL_FORMAT", solFormat);
    }

    if ( ! pbParams->isAttributeDefaultValue<NOMAD::ArrayOfDouble>("GRANULARITY") )
    {

        // Update SOL_FORMAT.
        // We want to remember the number of decimals in
        // GRANULARITY arguments, to be used later for formatting.
        //
        // Default is DISPLAY_PRECISION_STD.

        auto newSolFormat = setFormatFromGranularity( pbParams->getAttributeValue<NOMAD::ArrayOfDouble>("GRANULARITY") );
        setAttributeValue("SOL_FORMAT", newSolFormat);
    }
    
    // The default value is empty: set a basic display stats: BBE OBJ
    auto displayStats = getAttributeValueProtected<NOMAD::ArrayOfString>("DISPLAY_STATS",false);
    if ( displayStats.size() == 0 )
    {
        NOMAD::ArrayOfString aos("BBE OBJ");
        setAttributeValue("DISPLAY_STATS", aos );
    }

    
    /*------------------------------------------------------*/
    /* Stats file                                           */
    /*------------------------------------------------------*/
    
    auto statsFileParam = getAttributeValueProtected<NOMAD::ArrayOfString>("STATS_FILE",false) ;
    std::string statsFileName;
    if (statsFileParam.size() > 0)
    {
        statsFileName = statsFileParam[0];
        if (statsFileParam.size() == 1)
        {
            // Default stats: BBE OBJ.
            statsFileParam.add("BBE");
            statsFileParam.add("OBJ");
        }
    }
    
    
    
    // Update stats file name
    auto addSeedToFileNames = runParams->getAttributeValue<bool>("ADD_SEED_TO_FILE_NAMES");
    auto problemDir = runParams->getAttributeValue<string>("PROBLEM_DIR");
    if (!statsFileName.empty())
    {
        auto seed = runParams->getAttributeValue<int>("SEED");
        NOMAD::completeFileName(statsFileName, problemDir, addSeedToFileNames, seed);
        statsFileParam.replace(0, statsFileName);
        setAttributeValue("STATS_FILE", statsFileParam);
    }
    
    _toBeChecked = false;
    
}
// End checkAndComply()




NOMAD::ArrayOfDouble NOMAD::DisplayParameters::setFormatFromGranularity( const NOMAD::ArrayOfDouble & aod )
{
    size_t n = aod.size();
    NOMAD::ArrayOfDouble solFormat(n, NOMAD::DISPLAY_PRECISION_STD);
    
    // Use GRANULARITY as an ArrayOfDouble.
    size_t nbDecimals;
    for ( size_t i=0 ; i < n ; i++ )
    {
        if ( aod[i] > 0 )
        {
            nbDecimals = aod[i].nbDecimals( );
            solFormat.set(i, nbDecimals);
        }
    }
    return solFormat;

}




