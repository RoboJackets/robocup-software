#pragma once

#include <QAbstractSocket>

bool multicast_add(QAbstractSocket* socket, const char* addr);
