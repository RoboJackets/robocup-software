#include "Robot2015LqrLookupTable.hpp"
#include "TunableRobotParameters.hpp"

const LqrLookupTable<OfflineLqrController::KType> Robot2015LqrLookupTable(
    Robot2015LqrLookupTableValues, Robot2015LqrLookupTableNumEntries,
    Robot2015LqrLookupTableMinRotVel, Robot2015LqrLookupTableMaxRotVel);
