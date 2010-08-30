#pragma once

#include <vector>
#include <QStringList>
#include <google/protobuf/repeated_field.h>

void debugInit(const char *filename);
QStringList debugTrace(const google::protobuf::RepeatedField<google::protobuf::uint64> &trace);