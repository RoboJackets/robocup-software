#include <field_dimensions.h>

const FieldDimensions FieldDimensions::Single_FieldDimensions(
    6.050f, 4.050f, 0.250f, 0.010f, 0.700f, 0.180f, 0.160f, 1.000f, 2.000f,
    0.500f, 1.000f, 0.350f, 6.550f, 4.550f);

const FieldDimensions FieldDimensions::Double_FieldDimensions(
    9.000f, 6.000f, 0.700f, 0.010f, 1.000f, 0.180f, 0.160f, 1.000f, 2.000f,
    0.500f, 1.000f, 0.500f, 10.400f, 7.400f);

const FieldDimensions FieldDimensions::Default_Dimensions =
    FieldDimensions::Double_FieldDimensions;

FieldDimensions FieldDimensions::Current_Dimensions =
    FieldDimensions::Default_Dimensions;
